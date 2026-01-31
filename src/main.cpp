#include "main.h"


//declare devices
pros::Motor left_side_front (1, pros::v5::MotorGears::blue, pros::v5::MotorUnits::counts);
pros::Motor left_side_back (11, pros::v5::MotorGears::blue, pros::v5::MotorUnits::counts);
pros::Motor right_side_front (10, pros::v5::MotorGears::blue, pros::v5::MotorUnits::counts);
pros::Motor right_side_back (17, pros::v5::MotorGears::blue, pros::v5::MotorUnits::counts);
pros::Motor feed (20, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
pros::Motor top (19, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
pros::Motor middle (9, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
pros::ADIDigitalOut alligner (3);
pros::ADIDigitalOut flap (4);
pros::Controller master(pros::E_CONTROLLER_MASTER);


// Constants for odometry
const double wheelDiameter = 3.25; // inches
const double ticksPerRev = 300.0;
const double trackWidth = 11.5;    // inches between left/right wheels
const double inchesPerTick = (M_PI * wheelDiameter) / ticksPerRev;


// Odometry state variables
double x = 0.0;
double y = 0.0;
double theta = 0.0;
int prevLeftTicks = 0;
int prevRightTicks = 0;


// PID settings
double kP = 1.0;        // Start higher for movement
double kI = 0.01;       // Small integral
double kD = 0.5;        // Moderate derivative
double turnkP = 2.0;    // Higher for turning
double turnkI = 0.02;
double turnkD = 1.0;


// PID control variables
double error = 0;
double prevError = 0;
double derivative = 0;
double integral = 0;
double turnError = 0;
double turnPrevError = 0;
double turnDerivative = 0;
double turnIntegral = 0;


// Autonomous targets
double desiredDistanceInches = 0;
double desiredTurnDegrees = 0;
double desiredTurnRadians = 0;


// Control flags
bool enableDrivePID = true;
bool resetDriverSensors = false;
bool isMoving = false;


// Settling thresholds
const double DISTANCE_TOLERANCE = 0.5;  // inches
const double ANGLE_TOLERANCE = 2.0;     // degrees



const double inchesPerDegree = (M_PI * trackWidth) / 360.0;
double degreesToTicks(double degrees) {
    double inches = degrees * inchesPerDegree;
    return inches / inchesPerTick;
}


void Odometry() {
    while (true) {
        int leftTicks = ((left_side_back.position() + left_side_front.position())/2);
        int rightTicks = ((right_side_back.position() + right_side_front.position())/2);


        int deltaLeft = leftTicks - prevLeftTicks;
        int deltaRight = rightTicks - prevRightTicks;


        prevLeftTicks = leftTicks;
        prevRightTicks = rightTicks;


        double leftDist = deltaLeft * inchesPerTick;
        double rightDist = deltaRight * inchesPerTick;


        double deltaTheta = (rightDist - leftDist) / trackWidth;
        double avgDist = (leftDist + rightDist) / 2.0;
        theta += deltaTheta;
        x += avgDist * cos(theta);
        y += avgDist * sin(theta);

		pros::lcd::set_text(2, "X: " + std::to_string(x));
		pros::lcd::set_text(3, "Y: " + std::to_string(y));
		pros::lcd::set_text(4, "Theta: " + std::to_string(theta));


        pros::delay(10);
    }
}


void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "LIVETESTCODE");
	//screen test


	left_side_front.set_reversed(false);
    left_side_back.set_reversed(false);
    right_side_front.set_reversed(true);
    right_side_back.set_reversed(true);
	//makes all motors facing the same direction
	left_side_back.tare_position();
	left_side_front.tare_position();
	right_side_back.tare_position();
	right_side_front.tare_position();


	pros::Task odomTask(Odometry);
	pros::Task pidTask(drivePID);
}


void disabled() {}


void competition_initialize() {

}


int drivePID(){
	while (enableDrivePID){
		if (resetDriverSensors){
			resetDriverSensors = false;
			left_side_back.tare_position();
			left_side_front.tare_position();
			right_side_back.tare_position();
			right_side_front.tare_position();
			bool lateralClose = fabs(error) < 0.5;  // Within 0.5 inches
        	bool turnClose = fabs(turnError) < 10;  // Within 10 ticks
        
        	if (lateralClose && turnClose) {
            	// Close enough - stop motors
            	left_side_back.move(0);
            	left_side_front.move(0);
            	right_side_back.move(0);
            	right_side_front.move(0);
            	pros::delay(1000);
            	continue;
			}
		}


		//Lateral PID
		int leftMotorAvgPos = ((left_side_back.position() + left_side_front.position())/2);
		int rightMotorAvgPos = ((right_side_back.position() + right_side_front.position())/2);
		//gets position of both motors and averages them


		double averageTicks = ((leftMotorAvgPos + rightMotorAvgPos) / 2.0);
		double averageInches = (averageTicks * inchesPerTick);


		//Potential
		error = desiredDistanceInches - averageInches;
		//Derivative
		derivative = error - prevError;
		//Integral
		totalError += error;
		if (totalError > 1000) totalError = 1000;
        if (totalError < -1000) totalError = -1000;


		double lateralMotorPower = ((error * kP) + (derivative* kD) + (totalError * kI));


		//Turning PID
		int turnDiff = (((left_side_back.position() + left_side_front.position())/2) - ((right_side_back.position() + right_side_front.position())/2));


		//Potential
		turnError = desiredTurnTicks - turnDiff;

		//Derivative
		turnDerivative = turnError - turnPrevError;

		//Integral
		turnTotalError += turnError;
		if (turnTotalError > 1000) turnTotalError = 1000;
        if (turnTotalError < -1000) turnTotalError = -1000;

		double turnMotorPower = (turnError * turnkP) + (turnDerivative * turnkD) + (turnTotalError * turnkI);


		// Calculate raw powers
		double leftPower = lateralMotorPower + turnMotorPower;
		double rightPower = lateralMotorPower + turnMotorPower;
		// Limit to valid range FIRST (-127 to 127)
		if (leftPower > 127) leftPower = 127;
		if (leftPower < -127) leftPower = -127;
		if (rightPower > 127) rightPower = 127;
		if (rightPower < -127) rightPower = -127;
		// Convert to voltage
		int leftVoltage = leftPower * (12000.0 / 127.0);
		int rightVoltage = rightPower * (12000.0 / 127.0);
		// Apply voltage (optional limits for safety)
		if (leftVoltage > 12000) leftVoltage = 12000;
		if (leftVoltage < -12000) leftVoltage = -12000;
		if (rightVoltage > 12000) rightVoltage = 12000;
		if (rightVoltage < -12000) rightVoltage = -12000;
		// Apply to motors
		left_side_back.move_voltage(leftVoltage);
		left_side_front.move_voltage(leftVoltage);
		right_side_back.move_voltage(rightVoltage);
		right_side_front.move_voltage(rightVoltage);


		// Debug output
        pros::lcd::set_text(5, "Error: " + std::to_string(error));
        pros::lcd::set_text(6, "TurnErr: " + std::to_string(turnError));
        pros::lcd::set_text(7, "L: " + std::to_string(leftVoltage) + " R: " + std::to_string(rightVoltage));



		turnPrevError = turnError;
		prevError = error;
		pros::delay(20);
	}


	return 1;
}


void autonomous() {
	resetDriverSensors = false;


	desiredDistanceInches = 24;
	pros::delay(2000);
	desiredTurnDegrees = 0;
	pros::delay(2000);
	desiredDistanceInches = -24;
	pros::delay(2000);
	desiredTurnDegrees = 360;
	pros::delay(2000);
	desiredDistanceInches = 0;
	pros::delay(2000);
	desiredTurnDegrees = -360;

}


void opcontrol() {
	enableDrivePID = false;
	//no longer needed


    alligner.set_value(false);
    flap.set_value(false);
	//state for start of match pneumatics

    bool alll = false; //boolean for alligner
    bool flll = true; //boolean for flap


    while (true) {
        int vertical = master.get_analog(ANALOG_LEFT_Y);
        int horizontal = master.get_analog(ANALOG_RIGHT_X);
        int left = vertical + horizontal;
        int right = vertical - horizontal;
        left_side_front.move(left);
        left_side_back.move(left);
        right_side_front.move(right);
        right_side_back.move(right);
		//arcade drive 
		//left joystick forward/back
		//rigth joystick left/right







		
        
			//intake without score
			//outake for bottom mid goals
			//outake for top mid goals
			//outake for long goals goals
        




        


        if (master.get_digital_new_press(DIGITAL_A)) {
            alll = !alll;
            flll = !flll;
            alligner.set_value(alll);
            flap.set_value(flll);
			//invert boolean logic for sketchy fix for 1 working and 1 broken solenoid
        }


        if (master.get_digital_new_press(DIGITAL_B)) {
            alll = !alll;
            alligner.set_value(alll);
			//invert boolean logic (normal edition)
        }
        pros::delay(20);
    }

}