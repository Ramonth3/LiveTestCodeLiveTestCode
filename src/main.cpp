#include "main.h"
#include <cmath>
#include <algorithm>

// Motor declarations - using degrees
pros::Motor left_side_front(1, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
pros::Motor left_side_back(11, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
pros::Motor right_side_front(10, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
pros::Motor right_side_back(17, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);

// Other motors and devices
pros::Motor middleRight(20, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
pros::Motor middleLeft(18, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
pros::Motor top(19, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);

pros::adi::DigitalOut descore(7);
pros::adi::DigitalOut flap(8);
pros::adi::DigitalOut middle(6);

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Imu imu(9);

// Physical constants
const double WHEEL_DIAMETER = 3.25;
const double GEAR_RATIO = 600.0 / 450.0;
const double DEG_PER_INCH = (360.0 * GEAR_RATIO) / (M_PI * WHEEL_DIAMETER);
const int MAX_VOLTAGE = 12000;

// PID controller for forward/backward movement
class MovePID {
private:
    double kP, kI, kD;
    double integral = 0;
    double prevError = 0;
    double integralLimit = 5000;
    double outputLimit = maxVoltage;
    double deadband = 0.5; // inches

public:
    MovePID(double p = 0, double i = 0, double d = 0) : kP(p), kI(i), kD(d) {}

    double calculate(double error, double dt = 0.02) {
        // Apply deadband
        if (fabs(error) < deadband) {
            integral = 0;
            prevError = error;
            return 0;
        }

        integral += error * dt;
        
        // Anti-windup
        if (integral > integralLimit) integral = integralLimit;
        if (integral < -integralLimit) integral = -integralLimit;

        double derivative = (error - prevError) / dt;
        double output = (kP * error) + (kI * integral) + (kD * derivative);

        // Add minimum voltage to overcome static friction
        if (fabs(output) > 100) {
            double sign = (output > 0) ? 1.0 : -1.0;
            output += sign * 2000;
        }

        // Clamp output
        if (output > outputLimit) output = outputLimit;
        if (output < -outputLimit) output = -outputLimit;

        prevError = error;
        return output;
    }

    void reset() {
        integral = 0;
        prevError = 0;
    }
};

// PID controller for turning
class TurnPID {
private:
    double kP, kI, kD;
    double integral = 0;
    double prevError = 0;
    double integralLimit = 2000;
    double outputLimit = maxVoltage;
    double deadband = 1.0; // degrees

public:
    TurnPID(double p = 0, double i = 0, double d = 0) : kP(p), kI(i), kD(d) {}

    double calculate(double errorDegrees, double dt = 0.02) {
        // Convert to radians for calculation
        double error = errorDegrees * M_PI / 180.0;
        
        // Apply deadband
        if (fabs(errorDegrees) < deadband) {
            integral = 0;
            prevError = error;
            return 0;
        }

        integral += error * dt;
        
        // Anti-windup
        if (integral > integralLimit) integral = integralLimit;
        if (integral < -integralLimit) integral = -integralLimit;

        double derivative = (error - prevError) / dt;
        double output = (kP * error) + (kI * integral) + (kD * derivative);

        // Clamp output
        if (output > outputLimit) output = outputLimit;
        if (output < -outputLimit) output = -outputLimit;

        prevError = error;
        return output;
    }

    void reset() {
        integral = 0;
        prevError = 0;
    }
};

// Global instances
MovePID movePID(1000.0, 0.0, 90.0);
TurnPID turnPID(1500.0, 0.0, 110.0);
bool autonActive = false;

// Get current IMU heading (normalized to -180 to 180)
double getIMUHeading() {
    if (imu.is_calibrating()) return 0;
    
    double degrees = imu.get_rotation();
    
    // Normalize to -180 to 180
    while (degrees > 180) degrees -= 360;
    while (degrees < -180) degrees += 360;
    
    return degrees;
}

// Get average motor position in inches - CORRECTED FOR GEAR RATIO
double getAveragePosition() {
    double leftPos = (left_side_front.get_position() + left_side_back.get_position()) / 2.0;
    double rightPos = (right_side_front.get_position() + right_side_back.get_position()) / 2.0;
    
    // Convert degrees to inches - DIVIDE by correction factor
    // (motors need to spin more degrees, so each degree = less distance)
    double leftInches = leftPos / degreesPerInch;
    double rightInches = rightPos / degreesPerInch;
    
    return (leftInches + rightInches) / 2.0;
}

// Move forward/backward a specific distance
void moveDistance(double targetInches, double maxTimeMs = 9000) {
    movePID.reset();
    
    // Reset motor positions for relative movement
    left_side_front.tare_position();
    left_side_back.tare_position();
    right_side_front.tare_position();
    right_side_back.tare_position();
    
    const double tolerance = 0.3; // inches
    const int settleCycles = 10;
    int settleCount = 0;
    
    uint32_t startTime = pros::millis();
    double lastError = 0;
    
    // Calculate target in motor degrees
    double targetDegrees = targetInches * degreesPerInch;
    
    while (autonActive && (pros::millis() - startTime < maxTimeMs)) {
        // Get current position in inches
        double currentPos = getAveragePosition();
        double error = targetInches - currentPos;
        
        // Calculate PID output
        double voltage = movePID.calculate(error);
        
        // Apply to all motors (same direction for forward/backward)
        left_side_front.move_voltage(voltage);
        left_side_back.move_voltage(voltage);
        right_side_front.move_voltage(voltage);
        right_side_back.move_voltage(voltage);
        
        // Check if we've settled
        if (fabs(error) < tolerance) {
            settleCount++;
            if (settleCount >= settleCycles) {
                break;
            }
        } else {
            settleCount = 0;
        }
        
        pros::delay(20);
    }
    
    // Stop motors
    left_side_front.move_voltage(0);
    left_side_back.move_voltage(0);
    right_side_front.move_voltage(0);
    right_side_back.move_voltage(0);
    
    pros::delay(100); // Settling time
}

// Turn to a specific angle using IMU?
void turnToAngle(double targetDegrees, double maxTimeMs = 9000) {
    turnPID.reset();
    
    // Get starting angle
    double startAngle = getIMUHeading();
    double targetAngle = startAngle + targetDegrees;
    
    // Normalize target to -180 to 180
    while (targetAngle > 180) targetAngle -= 360;
    while (targetAngle < -180) targetAngle += 360;
    
    const double tolerance = 2.0; // degrees
    const int settleCycles = 20;
    int settleCount = 0;
    
    uint32_t startTime = pros::millis();
    
    while (autonActive && (pros::millis() - startTime < maxTimeMs)) {
        // Get current angle
        double currentAngle = getIMUHeading();
        
        // Calculate shortest path error
        double error = targetAngle - currentAngle;
        if (error > 180) error -= 360;
        if (error < -180) error += 360;
        
        // Calculate PID output
        double voltage = turnPID.calculate(error);
        
        // Apply voltage for turning (opposite directions)
        left_side_front.move_voltage(-voltage);
        left_side_back.move_voltage(-voltage);
        right_side_front.move_voltage(voltage);
        right_side_back.move_voltage(voltage);
        
        // Check if we've settled
        if (fabs(error) < tolerance) {
            settleCount++;
            if (settleCount >= settleCycles) {
                break;
            }
        } else {
            settleCount = 0;
        }
        
        pros::delay(20);
    }
    
    // Stop motors
    left_side_front.move_voltage(0);
    left_side_back.move_voltage(0);
    right_side_front.move_voltage(0);
    right_side_back.move_voltage(0);
    
    pros::delay(100); // Settling time
}

// Alternative approach: Simple multiplier for distance
void moveDistanceSimple(double targetInches, double maxTimeMs = 3000) {
    // Even simpler: multiply the target by the gear ratio
    double correctedTarget = targetInches * gearRatioCorrection;
    
    // Now use the existing logic with the corrected target
    moveDistance(correctedTarget, maxTimeMs);
}

void initialize() {
    // Configure motor directions
    left_side_front.set_reversed(true);
    left_side_back.set_reversed(true);
    right_side_front.set_reversed(false);
    right_side_back.set_reversed(false);
    
    // Set brake modes for autonomous
    left_side_front.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    left_side_back.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    right_side_front.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    right_side_back.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    
    // Reset motor positions
    left_side_front.tare_position();
    left_side_back.tare_position();
    right_side_front.tare_position();
    right_side_back.tare_position();
    
    // Calibrate IMU
    imu.reset();
    while (imu.is_calibrating()) {
        pros::delay(10);
    }
}

void disabled() {
    autonActive = false;
}

void competition_initialize() {
    autonActive = false;
}

void autonomous() {
    autonActive = true;
    
    // Test sequence - now should go actual 24 inches
    // The function already corrects for gear ratio internally
    //moveDistance(24.0);
	//moveDistance(-24.0);

	turnToAngle(90);
    
    // Or use the simple version:
    //moveDistanceSimple(24.0);
    
    autonActive = false;
}

void opcontrol() {
    // Switch to coast for driver control
    left_side_front.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    left_side_back.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    right_side_front.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    right_side_back.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    
    // Initialize pneumatics
    descore.set_value(false);
    flap.set_value(false);
    middle.set_value(true);
    
    bool dlll = false;
    bool flll = true;
    bool mlll = false;
    
    
    while (true) {
        // Arcade drive
        int vertical = master.get_analog(ANALOG_LEFT_Y);
        int horizontal = master.get_analog(ANALOG_RIGHT_X);
        
        int left = vertical + horizontal;
        int right = vertical - horizontal;
        
        left_side_front.move(left);
        left_side_back.move(left);
        right_side_front.move(right);
        right_side_back.move(right);
        
        // Intake/outtake control (unchanged)
        if (master.get_digital(DIGITAL_R1)) {
            middleRight.move_velocity(-600);
            middleLeft.move_velocity(600);
        } else if (master.get_digital(DIGITAL_R2)) {
            middleRight.move_velocity(600);
            middleLeft.move_velocity(-600);
            top.move_velocity(600);
        } else if (master.get_digital(DIGITAL_L1)) {
            top.move_velocity(-600);
            middleRight.move_velocity(-600);
            middleLeft.move_velocity(600);
        } else if (master.get_digital(DIGITAL_L2)) {
            top.move_velocity(-600);
            middleRight.move_velocity(-600);
            middleLeft.move_velocity(600);
        } else {
            top.move_velocity(0);
            middleRight.move_velocity(0);
            middleLeft.move_velocity(0);
        }
        
        // Pneumatic controls
        if (master.get_digital_new_press(DIGITAL_A)) {
            flll = !flll;
            flap.set_value(flll);
        }
        
        if (master.get_digital_new_press(DIGITAL_B)) {
            dlll = !dlll;
            descore.set_value(dlll);
        }
        
        if (master.get_digital_new_press(DIGITAL_Y)) {
            mlll = !mlll;
            middle.set_value(mlll);
        }
        
        pros::delay(20);
    }
}