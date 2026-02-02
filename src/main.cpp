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

			//determine accel direction
		if (fabs(error) < stoppingDistance) {
			// DE Accel phase
			profile.acceleration = -copysign(profile.maxAccel, profile.velocity);

		} else if (fabs(profile.velocity) < profile.maxVel) {
			// Accel phase
			profile.acceleration = copysign(profile.maxAccel, error);

		} else {
			//cruise phase
			profile.acceleration = 0;
		}

		//update velocity and pos
		profile.velocity += profile.acceleration * dt;
		if (fabs(profile.velocity) > profile.maxVel) {
			profile.velocity = copysign(profile.maxVel, profile.velocity);
		}

		profile.current += profile.velocity * dt;

	}

	void calculateVolt(double targetX, double targetY, double targetTheta, double currentX, double currentY, double currentTheta, double current_v_left, double current_v_right, double& leftVoltage, double& rightVoltage){

		// Pos error
		double dX = targetX - currentX;
		double dY = targetY - currentY;
		double distanceError = sqrt(dX*dX + dY*dY);

		// Desired heading to target
		double desiredHeading = atan2(dY, dX);

		// Heading error (shortest path)
		double headingError = desiredHeading - currentTheta;
		while (headingError > M_PI) headingError -= 2 * M_PI;
        while (headingError < -M_PI) headingError += 2 * M_PI;

		// Final angle error
        double angleError = targetTheta - currentTheta;
        while (angleError > M_PI) angleError -= 2 * M_PI;
        while (angleError < -M_PI) angleError += 2 * M_PI;

		// Update motion profiles
		linearProfile.target = 0; //obv we want distance 0, lol
		linearProfile.maxVel = maxLinearVel;
		linearProfile.maxAccel = maxLinearAccel;
		update_profile(linearProfile, 0.02);

		// Calc PID volts
		double linearVoltage = linearPID.calculate(distanceError);
        double angularVoltage = angularPID.calculate(angleError); // Did i not add angle error???? check later
        double headingVoltage = headingPID.calculate(headingError);

		//Feedforward volt (based on motion profile)
		double ffLinear = kv * linearProfile.velocity + ka * linearProfile.acceleration;
        double ffAngular = kav * angularProfile.velocity + kaa * angularProfile.acceleration;
        
        // Combine feedback and feedforward
        double totalLinear = linearVoltage + ffLinear;
        double totalAngular = angularVoltage + ffAngular + headingVoltage;
        
        // Differential drive conversion to motor voltages
        leftVoltage = totalLinear - totalAngular;
        rightVoltage = totalLinear + totalAngular;
        
        // Limit voltages
        double max_voltage = std::max(fabs(leftVoltage), fabs(rightVoltage));
        if (max_voltage > maxVoltage) {
            double scale = maxVoltage / max_voltage;
            leftVoltage *= scale;
            rightVoltage *= scale;
		}
	}

	void reset() {
        linearPID.reset();
        angularPID.reset();
        headingPID.reset();
        linearProfile = Profile{};
        angularProfile = Profile{};
	}

	void set_limits(double linearVel, double angularVel, 
                    double linearAccel, double angularAccel) {
        maxLinearVel = linearVel;
        maxAngularVel = angularVel;
        maxLinearAccel = linearAccel;
        maxAngularAccel = angularAccel;
    }
};


// Odometry Structure
struct Pos { 
	double x = 0, y = 0, theta = 0;
	double vLeft = 0, vRight = 0; //inch per sec
	Pos() = default;
	Pos(double x, double y, double theta) : x(x), y(y), theta(theta) {}
};





void initialize() {
	// pros::lcd::initialize();


	//left_side_front.set_reversed(false);
    //left_side_back.set_reversed(false);
    //right_side_front.set_reversed(true);
    //right_side_back.set_reversed(true);

}


void disabled() {}


void competition_initialize() {

}


void autonomous() {

}


void opcontrol() {
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