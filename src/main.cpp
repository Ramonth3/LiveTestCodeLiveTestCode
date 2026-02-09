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

// PID values (TUNE LATER just random values)
double kP_move = 700;
double kI_move = 0.0;
double kD_move = 220;

double kP_turn = 95;
double kI_turn = 0.0;
double kD_turn = 520;

double kP_heading = 60;

// struct ODOM
struct Odometry {
    double x = 0;
    double y = 0;
    double heading = 0;

    double left_in = 0;
    double right_in = 0;
} odom;

// Helpers
double degToIn(double deg) {
    return deg / DEG_PER_INCH;
}
double getHeading() {
    if (imu.is_calibrating()) return odom.heading;
    double h = fmod(imu.get_rotation(), 360.0);
    if (h < 0) h += 360.0;
    return h;
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

    double dL = left_now - odom.left_in;
    double dR = right_now - odom.right_in;
    double dC = (dL + dR) / 2.0;

    odom.heading = getHeading();
    double rad = odom.heading * M_PI / 180.0;

    odom.x += dC * cos(rad);
    odom.y += dC * sin(rad);

    odom.left_in = left_now;
    odom.right_in = right_now;
}

// ODOM + PID
void moveDistance(double inches, int timeout = 3000) {
    left_side_front.tare_position();
    left_side_back.tare_position();
    right_side_front.tare_position();
    right_side_back.tare_position();

    odom.left_in = 0;
    odom.right_in = 0;

    updateOdometry();

    double start_x = odom.x;
    double start_y = odom.y;
    double start_h = odom.heading * M_PI / 180.0;

    double target_x = start_x + inches * cos(start_h);
    double target_y = start_y + inches * sin(start_h);

    double last_error = 0;
    double integral = 0;

    int start = pros::millis();

    while (pros::millis() - start < timeout) {
        updateOdometry();

        double dx = target_x - odom.x;
        double dy = target_y - odom.y;
        double dist_error = sqrt(dx * dx + dy * dy);

        if (dist_error < 0.3) break;

        double derivative = dist_error - last_error;
        last_error = dist_error;

        if (fabs(dist_error) < 8)
            integral += dist_error;

        double power =
            dist_error * kP_move +
            integral * kI_move +
            derivative * kD_move;

        if (power > MAX_VOLTAGE) power = MAX_VOLTAGE;
		if (power < -MAX_VOLTAGE) power = -MAX_VOLTAGE;

        double target_heading = atan2(dy, dx) * 180.0 / M_PI;
        double heading_error = target_heading - odom.heading;
        if (heading_error > 180) heading_error -= 360;
        if (heading_error < -180) heading_error += 360;

        double correction = heading_error * kP_heading;

        setDriveVoltage(power - correction, power + correction);
        pros::delay(10);
    }

    setDriveVoltage(0, 0);
}

void turnToAngle(double target, int timeout = 2000) {
    double last_error = 0;
    double integral = 0;

    int start = pros::millis();

    while (pros::millis() - start < timeout) {
        double heading = getHeading();
        double error = target - heading;

        if (error > 180) error -= 360;
        if (error < -180) error += 360;

        if (fabs(error) < 1.0) break;

        double derivative = error - last_error;
        last_error = error;

        if (fabs(error) < 20)
            integral += error;

        double power =
            error * kP_turn +
            integral * kI_turn +
            derivative * kD_turn;

        if (power > MAX_VOLTAGE) power = MAX_VOLTAGE;
		if (power < -MAX_VOLTAGE) power = -MAX_VOLTAGE;

        setDriveVoltage(-power, power);
        pros::delay(10);
    }

    setDriveVoltage(0, 0);
}

void turnRelative(double deg, int timeout = 2000) {
    double target = getHeading() + deg;
    target = fmod(target, 360.0);
    if (target < 0) target += 360.0;
    turnToAngle(target, timeout);
}

void initialize() {
    // Configure motor directions
    left_side_front.set_reversed(true);
    left_side_back.set_reversed(true);
    right_side_front.set_reversed(false);
    right_side_back.set_reversed(false);
    
    // Set brake modes for autonomous
    left_side_front.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    left_side_back.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    right_side_front.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    right_side_back.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    imu.reset();
    pros::delay(2000);

    odom.heading = getHeading();
}

void disabled() {
}

void competition_initialize() {
}

void autonomous() {
	moveDistance(24);
    turnRelative(90);
    moveDistance(12);
    turnRelative(-90);
    moveDistance(-24);
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