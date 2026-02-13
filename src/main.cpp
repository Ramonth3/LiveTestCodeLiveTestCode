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

pros::adi::DigitalOut DoublePark(6);
pros::adi::DigitalOut descore(7);
pros::adi::DigitalOut scrapper(8);

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Imu imu(9);

// Physical constants
const double WHEEL_DIAMETER = 3.25;
const double GEAR_RATIO = 600.0 / 450.0;
const double DEG_PER_INCH = (360.0 * GEAR_RATIO) / (M_PI * WHEEL_DIAMETER);
const int MAX_VOLTAGE = 12000;

// PID values (TUNE LATER just random values)
double kP_move = 900;
double kI_move = 0;
double kD_move = 200;

double kP_turn = 275;
double kI_turn = 0;
double kD_turn = 10;

double kP_heading = 0;

// ODOM
double global_x = 0;
double global_y = 0;
double global_heading = 0;

double local_x = 0;
double local_y = 0;

double prev_left_in = 0;
double prev_right_in = 0;

// Helpers
double degToIn(double deg) {
    return deg / DEG_PER_INCH;
}
double getHeading() {
    if (imu.is_calibrating()) return 0;
    return imu.get_rotation();
}
void setDriveVoltage(double left, double right) {
    left_side_front.move_voltage(left);
    left_side_back.move_voltage(left);
    right_side_front.move_voltage(right);
    right_side_back.move_voltage(right);
}
double getLeftDeg() {
    return (left_side_front.get_position() + left_side_back.get_position()) / 2.0;
}
double getRightDeg() {
    return (right_side_front.get_position() + right_side_back.get_position()) / 2.0;
}

// updates ODOM
void updateOdometry() {
    double left_now = degToIn(getLeftDeg());
    double right_now = degToIn(getRightDeg());

    double dL = left_now - prev_left_in;
    double dR = right_now - prev_right_in;

    prev_left_in = left_now;
    prev_right_in = right_now;

    double dC = (dL + dR) / 2.0;

    global_heading = getHeading();
    double rad = global_heading * M_PI / 180.0;

    double dx = dC * cos(rad);
    double dy = dC * sin(rad);

    global_x += dx;
    global_y += dy;

    local_x += dx;
    local_y += dy;
}


// ODOM + PID
void moveDistance(double inches, int timeout = 3000) {

    // Reset motor encoders
    left_side_front.tare_position();
    left_side_back.tare_position();
    right_side_front.tare_position();
    right_side_back.tare_position();

    prev_left_in = 0;
    prev_right_in = 0;

    updateOdometry();

    double start_x = global_x;
    double start_y = global_y;
    double start_heading = getHeading();

    double last_error = 0;
    double integral = 0;

    int start_time = pros::millis();

    while (pros::millis() - start_time < timeout) {

        updateOdometry();

        // Project movement onto starting heading
        double dx = global_x - start_x;
        double dy = global_y - start_y;

        double start_rad = start_heading * M_PI / 180.0;

        double traveled = dx * cos(start_rad) + dy * sin(start_rad);

        double error = inches - traveled;

        // Stop condition
        if (fabs(error) < 0.05){
            break;
        }
        // PID
        double derivative = error - last_error;
        last_error = error;

        if (fabs(error) < 6){
            integral += error;
        }else{
            integral = 0;
        }
            
        double power = error * kP_move + integral * kI_move + derivative * kD_move;

        // Heading correction
        double heading_error = start_heading - getHeading();

        while (heading_error > 180) heading_error -= 360;
        while (heading_error < -180) heading_error += 360;

        double turn_correction = heading_error * kP_heading;

        // Clamp
        if (power > MAX_VOLTAGE) power = MAX_VOLTAGE;
        if (power < -MAX_VOLTAGE) power = -MAX_VOLTAGE;

        setDriveVoltage(power - turn_correction, power + turn_correction);

        pros::delay(5);
    }

    setDriveVoltage(0, 0);
}




void turnToAngle(double degrees, int timeout = 6000) {

    double startHeading = imu.get_rotation();
    double target = startHeading + degrees;

    double error = 0;
    double lastError = 0;
    double integral = 0;

    int settleTime = 0;
    int startTime = pros::millis();

    while (pros::millis() - startTime < timeout) {

        double current = imu.get_rotation();
        error = target - current;

        // Wrap to shortest path
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        // Integral only near target (prevents windup)
        if (fabs(error) < 10){
            integral += error;
        }else{
            integral = 0;
        }

        double derivative = error - lastError;
        lastError = error;

        double power = (error * kP_turn) + (integral * kI_turn) + (derivative * kD_turn);

        // Clamp
        if (power > MAX_VOLTAGE) power = MAX_VOLTAGE;
        if (power < -MAX_VOLTAGE) power = -MAX_VOLTAGE;

        setDriveVoltage(power, -power);

        // Settling condition
        if (fabs(error) < 0.5) {
            break;
        }

        pros::delay(5);
    }
    setDriveVoltage(0, 0);
    pros::delay(200);
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

    imu.reset();
    while (imu.is_calibrating()) {
        pros::delay(20);
    }
    pros::delay(200);
}

void disabled() {
}

void competition_initialize() {
}

void autonomous() {
    turnToAngle(90);
    turnToAngle(90);
    turnToAngle(90);
    turnToAngle(90);


    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Heading: %.2f", imu.get_rotation());
	pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Global X: %.2f, Global Y: %.2f", global_x, global_y);
    pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Global X: %.2f, Global Y: %.2f", local_x, local_y);
    pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Temp ML: %.2f, Temp MR: %.2f, Temp Top: %.2f", middleLeft.get_temperature(), middleRight.get_temperature(), top.get_temperature());
    pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Temp LF: %.2f, Temp LB: %.2f, Temp RF: %.2f, Temp RB: %.2f", left_side_front.get_temperature(), left_side_back.get_temperature(), right_side_front.get_temperature(), right_side_back.get_temperature());
    pros::screen::print(pros::E_TEXT_MEDIUM, 6, "Temp RF: %.2f, Temp RB: %.2f", right_side_front.get_temperature(), right_side_back.get_temperature());
	
}

void opcontrol() {
    // Switch to coast for driver control
    left_side_front.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    left_side_back.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    right_side_front.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    right_side_back.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);	

    // Initialize pneumatics
    descore.set_value(false);
    scrapper.set_value(false);
    DoublePark.set_value(true);
    
    bool dlll = false; // descore
    bool slll = true; // scrapper
    bool plll = false; // double park (p for park)
    
    
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
            slll = !slll;
            scrapper.set_value(slll);
        }
        
        if (master.get_digital_new_press(DIGITAL_B)) {
            dlll = !dlll;
            descore.set_value(dlll);
        }
        
        if (master.get_digital_new_press(DIGITAL_Y)) {
            plll = !plll;
            DoublePark.set_value(plll);
        }
        
        pros::delay(20);
    }
}