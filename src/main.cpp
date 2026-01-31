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

pros::Imu imu_sensor(18); //reminder to change later to whatever is convinet



// Robot Physical Constants for odometry
const double wheelDiameter = 3.25; // inches
const double trackWidth = 11.5;    // inches between left/right wheels
const double middleWidth = trackWidth/2; //inches (center)
const double ticksPerRev = 300.0; //blue motor
const double inchesPerTick = (M_PI * wheelDiameter) / ticksPerRev;
const double maxVoltage = 12000.0; //mV
const double baseVoltage = 4000.0; // min to overcome friction

// PID Controls
class voltagePID {
private:
	double kP, kI, kD;
	double integral = 0;
	double prevError = 0;
	double integralLimit;
	double outputLimit;
	double deadBand = 100.0; //mV

public:
	//seting up avalues
	voltagePID(double p = 0, double i = 0, double d = 0, double iLimit = 2000, double outputLim = maxVoltage) : kP(p), kI(i), kD(d), integralLimit(iLimit), outputLimit(outputLim) {}
	
	//voltage output based on error
	double calculate(double error, double dt = 0.02) {
		integral += error *dt;

		//anti-windup
		if (integral > integralLimit) integral = integralLimit;
		if (integral < -integralLimit) integral = -integralLimit;

		double derivative = (error - prevError) / dt;

		//calc voltage
		double voltage = (kP * error) + (kI * integral) + (kD * derivative);

		//if moving add base voltage to overcome static friction
		if(fabs(voltage) > 0){
			double direction = (voltage > 0) ? 1.0 : 1.0;
			voltage += direction * baseVoltage;
		}

		//output limits
		if (voltage > outputLimit) voltage = outputLimit;
		if (voltage < outputLimit) voltage = -outputLimit;

		//deadband
		if(fabs(voltage) < deadBand) voltage = 0;

		prevError = error;

		return voltage;
	}

	void reset(){
		integral = 0;
		prevError = 0;
	}

	void set_gains(double p, double i, double d){
		kP = p;
		kI = i;
		kD = d;
	}

};


class PosControl {
private:

	voltagePID linearPID; // linaer motion inch to mV
	voltagePID angularPID; // rotation radian to mV
	voltagePID headingPID; // heading radians to mV

	double kv = 300.0; // Voltage per inch/sec
	double ka = 50.0; // Voltage per inch/sec²
	double kav = 200.0; // Voltage per rad/sec
	double kaa =  30.0; // Voltage per rad/sec²

	double maxLinearAccel = 48.0; // inches/sec²
	double maxAngularAccel = 360.0; // degrees/sec²
	double maxLinearVel = 36.0; // inches/sec
	double maxAngularVel = 360.0; // degrees/sec

	struct Profile {
		double target;
		double current;
		double velocity;
		double acceleration;
		double maxVel;
		double maxAccel;
		bool complete = false;
	} linearProfile, angularProfile;

public:
	PosControl() : 
		linearPID(800.0, 5.0, 150.0, 3000, maxVoltage),
		angularPID(5000.0, 10.0, 800.0, 2000, maxVoltage),
		headingPID(3000.0, 2.0, 400.0, 1000, maxVoltage) {}

	void update_profile(Profile& profile, double dt){
		if (profile.complete) return;
		double error = profile.target - profile.current;
		double stoppingDistance = (profile.velocity * profile.velocity) / (2 * profile.maxAccel);

		if (fabs(error) <= 0.1) {
			profile.velocity = 0;
			profile.current = profile.target;
			profile.complete = true;
			return;
		}
		
	}


	

};

//ignore for now


// Odometry Structure
struct Pos { 
	double x = 0, y = 0, theta = 0;
	double vLeft = 0, vRight = 0; //inch per sec
	Pos() = default;
	Pos(double x, double y, double theta) : x(x), y(y), theta(theta) {}
};








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
//just wrong
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
}


void disabled() {}


void competition_initialize() {


		// Debug output
        //pros::lcd::set_text(5, "Error: " + std::to_string(error));
        //pros::lcd::set_text(6, "TurnErr: " + std::to_string(turnError));
        //pros::lcd::set_text(7, "L: " + std::to_string(leftVoltage) + " R: " + std::to_string(rightVoltage));
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