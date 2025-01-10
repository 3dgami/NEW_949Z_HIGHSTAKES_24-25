#include "main.h"
//#include "autoSelect/selection.h"
#include "Master-Selector/api.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <cstddef>



//update all motor ports if needed
pros::Controller master{CONTROLLER_MASTER};	
pros::Motor right_front(10, pros::E_MOTOR_GEAR_600);
pros::Motor left_front(-11, pros::E_MOTOR_GEAR_600);
pros::Motor left_back(-4, pros::E_MOTOR_GEAR_600);
pros::Motor right_back(9, pros::E_MOTOR_GEAR_600);
pros::Motor right_mid(20, pros::E_MOTOR_GEAR_600);
pros::Motor left_mid(-1, pros::E_MOTOR_GEAR_600);
pros::Motor Conveyor(3);
pros::Motor Intake(12);
pros::MotorGroup driveL_train({-11, -1, -4});//UPDATE WITH MOTOR WIRING CHANGING
pros::MotorGroup driveR_train({10, 20, 9});
pros::MotorGroup full_drivetrain({-11, -1, -4, 10, 20, 9});
pros::Motor LadyBrown(18);
pros::Rotation LadyBrownRotate(14);
pros::Optical Color_sensor(19);
pros::IMU imu(13);
pros::Rotation RotationX(100, false);
pros::Rotation RotationY(200, true);

bool ExpansionClampState;
bool ExpansionIntakeState;
bool ExpansionNeutral;
bool DoinkerState;
bool LadyBrownState;

pros::ADIDigitalOut ExpansionIntake('A');
pros::ADIDigitalOut ExpansionClamp('B');
pros::ADIDigitalOut Doinker('C');


// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&RotationX, lemlib::Omniwheel::NEW_2, 4.65);

// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&RotationY, lemlib::Omniwheel::NEW_2, -2.5);


pros::MotorGroup driveL_trainLem({-11, -12});//UPDATE WITH MOTOR WIRING CHANGING
pros::MotorGroup driveR_trainLem({10, 1});

// drivetrain settings //UPDATE
lemlib::Drivetrain drivetrain(&driveL_trainLem, // left motor group
                              &driveR_trainLem, // right motor group
                              14, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3,25" omnis
                              300, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings lateral_controller(15, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, //&vertical_tracking_wheel, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal_tracking_wheel, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

void SetDriveRelative(int ticks, int Lspeed, int Rspeed)
	{

		left_front.move_relative(-(ticks), Lspeed);
		left_back.move_relative(-(ticks), Lspeed);
		left_mid.move_relative(-(ticks), Lspeed);
		right_front.move_relative(ticks, Rspeed);
		right_back.move_relative(ticks, Rspeed);
		right_mid.move_relative(ticks, Rspeed);
	}

void SetDrive(int Lspeed, int Rspeed)
	{
		left_mid.move(-(Lspeed));
		left_front.move(-(Lspeed));
		left_back.move(-(Lspeed));
		right_front.move(Rspeed);
		right_back.move(Rspeed);
		right_mid.move(Rspeed);
	}

double getLeftPos()
{
	return -(left_back.get_position() + left_front.get_position()) / 2;
}

double getRightPos()
{
	return (right_front.get_position() + right_back.get_position()) / 2;
}

double getPos()
{
	return (getLeftPos() + getRightPos()) / 2;
}

void driveTrain(int distance, int timeout)
{

	driveL_train.set_reversed(true);
	int startPos = getPos();
	double kp = 10.00;
	double ki = 1.0;
	double kd = -10.50;   /*derivitive should control and stop overshooting this can be done
						  by having kd be negative or having a (P + I - D) for the output PS 
						*/
	double P;
	double I = 0;
	double D;
	int lastError = 0;
	int errorTerm = 0;
	int errorTotal = 0;
	int sign;
	int count = 0;
	int SERT;
	int SERT_count = 0;
	bool SERT_bool = false;

	sign = (distance < 0) ? -1 : 1;
	
	errorTerm = distance + startPos - getPos();

	while (errorTerm > 1 or errorTerm < -1 and count < timeout) // and SERT_count < SERTx
	{
		if(count > timeout or SERT_count > SERT)
		{
			break;
			printf("TIMEOUT \n");
		}

		errorTerm = distance + startPos - getPos();

		int Pos = getPos();

		errorTotal = errorTotal + errorTerm;

		sign = (errorTerm < 0) ? -1 : 1;


		errorTotal = (errorTotal > 500 / ki) ? 500 / ki : errorTotal;

		P = errorTerm * kp;
		//I = errorTotal * ki;
		D = (lastError - errorTerm) * kd;
		double output = (((P + I + D) + (1000 * sign)));

		printf("O=%0.2f, P=%0.2f, D=%0.2f, Position=%d, startPos=%d Err=%d\n",output, P, D, Pos, startPos, errorTerm);

		driveL_train.move_voltage(output);
		driveR_train.move_voltage(output);
		//full_drivetrain.move_voltage(output);
		

		lastError = errorTerm;
		pros::delay(20);
		
		//SERT_bool = (errorTerm < 15) ? true : false;
		//SERT_count = (SERT_bool = true) ? SERT_count + 20 : SERT_count;
		count += 20;

	}
	driveL_train.move_voltage(0);
	driveR_train.move_voltage(0);
	printf("End\nErr=%d", errorTerm);
	driveL_train.set_reversed(false);

	return;
}

void turn(int angle)
{
	driveL_train.set_reversed(false);
	double CircleTicks = 2750;
	int turnTicks = (CircleTicks/360) * angle;
	int count = 0;



	int startPos = getPos();
	double kp = 11.0;
	double ki = 0.1;
	double kd = -5.50; /*derivitive should control and stop overshooting this can be done
						  by having kd be negative or having a (P + I - D) for the output
						*/
	double P;
	double I;
	double D;
	int lastError = 0;
	int errorTerm;
	int errorTotal = 0;
	int sign = 1;

	sign = (angle > 0) ? 1 : -1;

	errorTerm = (turnTicks + startPos) - floor(getPos());


	printf("start\n");
	while (errorTerm > 1 or errorTerm < -1 and count <= 2000)
	{

		if(count > 2000)
		{
			break;
			printf("TIMEOUT \n");
		}

		errorTerm = (turnTicks + startPos) - floor(getPos());

		sign = (errorTerm < 0) ? -1 : 1;

		int pos = getPos();

		errorTotal = errorTotal + errorTerm;

		errorTotal = (errorTotal > 500 / ki) ? 500 / ki : errorTotal;


		P = errorTerm * kp;
		I = errorTotal * ki;
		D = (lastError - errorTerm) * kd;
		int output = (((P + D)) + (1500 * sign));


		printf("step err=%d, P=%.02f, D=%.02f, StartPos=%d, Pos=%d, O=%d turn=%d count=%d \n", errorTerm, P, D, startPos, pos, output, turnTicks, count);


		driveL_train.move_voltage(output);
		driveR_train.move_voltage(output);

		lastError = errorTerm;
		pros::delay(10);
		count += 10;
	}
	driveL_train.move_voltage(0);
	driveR_train.move_voltage(0);
	driveL_train.set_reversed(true);

	pros::delay(10);
	printf("\nDone err=%d\n, O=%d", errorTerm, turnTicks);

	return;
}

void Centralturn(int angle, bool side)
{	
	driveL_train.set_reversed(true);
	double CircleTicks = 5400;
	int turnTicks = (CircleTicks/360) * angle;
	int count = 0;
	int startPos;
	double Pos;

	double kp = 2.0;
	double ki = 0.1;
	double kd = -5.50; /*derivitive should control and stop overshooting this can be done
						  by having kd be negative or having a (P + I - D) for the output
						*/
	double P;
	double I;
	double D;
	int lastError = 0;
	int errorTerm;
	int errorTotal = 0;
	int sign = 1;

	sign = (angle > 0) ? 1 : -1;


	if(side == 1)
	{
		startPos = getRightPos();
	}
	
	else if(side == 0)
	{
		startPos = getLeftPos();
	}
	Pos = (side == 1) ? getRightPos() : getLeftPos();

	errorTerm = (turnTicks + startPos) - Pos;

	while(errorTerm > 1 or errorTerm < 1 and count <= 3000)
	{
		if(count > 3000)
		{
			break;
			printf("TIMEOUT \n");
		}

		Pos = (side == 1) ? getRightPos() : getLeftPos();
		errorTerm = (turnTicks + startPos) - floor(Pos);

		sign = (errorTerm < 0) ? -1 : 1;

		int pos = getPos();

		errorTotal = errorTotal + errorTerm;

		errorTotal = (errorTotal > 500 / ki) ? 500 / ki : errorTotal;


		P = errorTerm * kp;
		I = errorTotal * ki;
		D = (lastError - errorTerm) * kd;
		int output = (((P + D)) + (1250 * sign));


		printf("step err=%d, P=%.02f, D=%.02f, StartPos=%d, Pos=%d, O=%d turn=%d count=%d \n", errorTerm, P, D, startPos, pos, output, turnTicks, count);

		if(side == 1)
		{
			driveR_train.move_voltage(output);
		}
		else if(side == 0)
		{
			driveL_train.move_voltage(output);
		}
		
		lastError = errorTerm;
		pros::delay(10);
		count += 10;
	}
	driveL_train.move_voltage(0);
	driveR_train.move_voltage(0);

	pros::delay(10);
	printf("\nDone err=%d\n, O=%d", errorTerm, turnTicks);

	return;
}

void gyroTurn(int angle)
{
	printf("start \n");
	//driveL_train.set_reversed(true);
	//driveR_train.set_reversed(true);

	double heading = imu.get_heading();

	double kp = 20.0;
	double ki = 0.1;
	double kd = -19; /*derivitive should control and stop overshooting this can be done
						  by having kd be negative or having a (P + I - D) for the output
						*/
	double P;
	double I;
	double D;
	int lastError = 0;
	double errorTerm;
	int errorTotal = 0;
	int sign = 1; 
	int count = 0;
	double ActualAngle;

	ActualAngle = (heading + angle) > 360 ? heading + angle - 360 : heading + angle;
	double diff = fabs(heading - ActualAngle);
	if(diff <= 180)
	{
		//errorTerm = -(360 + diff);
		errorTerm = diff;
		sign = (errorTerm < 0) ? sign = -1 : sign = 1;

	}
	else if(diff > 180)
	{
		errorTerm = -diff;
		sign = (errorTerm < 0) ? sign = -1 : sign = 1;
	}
	/*else
	{
		errorTerm = -diff;
		sign = (errorTerm < 0) ? sign = -1 : sign = 1;
	}*/
	//errorTerm = diff;
	errorTerm = fabs(errorTerm);


	printf("start\n");
	while (errorTerm > 0.5 or errorTerm < -0.5 and count <= 6000) 
	{

		if(count > 6000)
		{
			break;
			printf("TIMEOUT \n");
		}

		heading = imu.get_heading();
		diff = heading - angle;
		if(diff <= 180)
		{
		//errorTerm = -(360 + diff);
			errorTerm = diff;
			sign = (errorTerm < 0) ? sign = -1 : sign = 1;

		}
		else if(diff > 180)
		{
			errorTerm = -diff;
			sign = (errorTerm < 0) ? sign = -1 : sign = 1;
		}
		/*else
		{
			errorTerm = -diff;
			sign = (errorTerm < 0) ? sign = -1 : sign = 1;
		}*/
		//errorTerm = diff;
		errorTerm = fabs(errorTerm);

		errorTotal = errorTotal + errorTerm;

		errorTotal = (errorTotal > 500 / ki) ? 500 / ki : errorTotal;


		P = errorTerm * kp;
		I = errorTotal * ki;
		D = (lastError - errorTerm) * kd;
		int output = (((P + D) + 1500) * sign);


		printf("err=%0.2f, P=%.02f, D=%.02f, O=%d, heading=%0.2f count=%d \n", errorTerm, P, D, output, heading, count);


		driveL_train.move_voltage(output);
		driveR_train.move_voltage(output);

		lastError = errorTerm;
		pros::delay(10);
		count += 10;
	}
	driveL_train.move_voltage(0);
	driveR_train.move_voltage(0);
	driveL_train.set_reversed(true);
	driveR_train.set_reversed(false);

	pros::delay(100);
	printf("Heading=%0.2f \n", imu.get_heading());

	return;
}

void AbsGyroTurn(int angle)
{
	printf("start \n");
	driveL_train.set_reversed(true);
	driveR_train.set_reversed(true);

	double heading = imu.get_heading();

	double kp = 40.0;
	double ki = 0.1;
	double kd = -20.50; /*derivitive should control and stop overshooting this can be done
						  by having kd be negative or having a (P + I - D) for the output
						*/
	double P;
	double I;
	double D;
	int lastError = 0;
	double errorTerm;
	int errorTotal = 0;
	int sign = 1; 
	int count = 0;

	double diff = heading - angle;
	if(diff < -180)
	{
		errorTerm = -(360 + diff);
		sign = (errorTerm < 0) ? sign = -1 : sign = 1;
	}
	else if(diff > 180)
	{
		errorTerm = 360 - diff;
		sign = (errorTerm < 0) ? sign = -1 : sign = 1;
	}
	else
	{
		errorTerm = -diff;
		sign = (errorTerm < 0) ? sign = -1 : sign = 1;
	}
	errorTerm = fabs(errorTerm);


	printf("start\n");
	while (errorTerm > 0.5 or errorTerm < -0.5 and count <= 2000) 
	{

		if(count > 2000)
		{
			break;
			printf("TIMEOUT \n");
		}

		heading = imu.get_heading();
		diff = heading - angle;
		if(diff < -180)
		{
			errorTerm = -(360 + diff);
			sign = (errorTerm < 0) ? sign = -1 : sign = 1;
		}
		else if(diff > 180)
		{
			errorTerm = 360 - diff;
			sign = (errorTerm < 0) ? sign = -1 : sign = 1;
		}
		else
		{
			errorTerm = -diff;
			sign = (errorTerm < 0) ? sign = -1 : sign = 1;
		}
		errorTerm = fabs(errorTerm);

		errorTotal = errorTotal + errorTerm;

		errorTotal = (errorTotal > 500 / ki) ? 500 / ki : errorTotal;


		P = errorTerm * kp;
		I = errorTotal * ki;
		D = (lastError - errorTerm) * kd;
		int output = (((P + D) + 1500) * sign);


		printf("err=%0.2f, P=%.02f, D=%.02f, O=%d, heading=%0.2f count=%d \n", errorTerm, P, D, output, heading, count);


		driveL_train.move_voltage(output);
		driveR_train.move_voltage(output);

		lastError = errorTerm;
		pros::delay(10);
		count += 10;
	}
	driveL_train.move_voltage(0);
	driveR_train.move_voltage(0);
	driveL_train.set_reversed(true);
	driveR_train.set_reversed(false);

	pros::delay(100);
	printf("Heading=%0.2f \n", imu.get_heading());

	return;
}

void LadyBrownArm(int position, int timeout)
{

	double kp = 1.0;
	double ki = 1.0;
	double kd = -2.5;   /*derivitive should control and stop overshooting this can be done
						  by having kd be negative or having a (P + I - D) for the output PS 
						*/
	double P;
	double I = 0;
	double D;
	int lastError = 0;
	int errorTerm = 0;
	int errorTotal = 0;
	int sign;
	int count = 0;
	int SERT;
	int SERT_count = 0;
	bool SERT_bool = false;

	errorTerm = position - LadyBrownRotate.get_position();

	while (errorTerm > 10 or errorTerm < -10 and count < timeout) // and SERT_count < SERTx
	{
		if(count > timeout or SERT_count > SERT)
		{
			break;
			printf("TIMEOUT \n");
		}

		errorTerm = position - LadyBrownRotate.get_position();

		sign = (errorTerm < 0) ? sign = -1 : sign = 1;

		int Pos = LadyBrownRotate.get_angle();

		errorTotal = errorTotal + errorTerm;

		sign = (errorTerm < 0) ? -1 : 1;


		errorTotal = (errorTotal > 500 / ki) ? 500 / ki : errorTotal;

		P = errorTerm * kp;
		//I = errorTotal * ki;
		D = (lastError - errorTerm) * kd;
		double output = ((P + D) + (2000 * sign));

		printf("O=%0.2f, P=%0.2f, D=%0.2f, Position=%d, Err=%d, rotationPOs=%d \n",output, P, D, Pos, errorTerm, LadyBrownRotate.get_position());

		LadyBrown.move_voltage(-output);
		
		

		lastError = errorTerm;
		pros::delay(10);
		
		//SERT_bool = (errorTerm < 15) ? true : false;
		//SERT_count = (SERT_bool = true) ? SERT_count + 20 : SERT_count;
		count += 20;

	}
	LadyBrown.move_voltage(0);
	printf("End\nErr=%d", errorTerm);

	return;
}

void LadyBrownTask()
{
	while(true)
	{
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
		{
			LadyBrownArm(48500, 2000);
			LadyBrownState = true;
		}

		//LADYBROWN HOLD
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
		{
			LadyBrownArm(56000, 2000); 
			LadyBrownState = false;
		}

		//LADYBROWN SCORE
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
		{
			LadyBrownArm(111000, 2000); 
			LadyBrownState = false;
		}

		//LADYBROWN STOW
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
		{
			LadyBrownArm(31000, 2000); 
			LadyBrownState = false;
		}
		pros::delay(10);

	}

}

//AUTON//

//RIGHT SIDE AUTON RED AWP
ASSET(R1RED_txt);
ASSET(R2RED_txt);
ASSET(R3RED_txt);

//RIGHT SIDE AUTON RED ELIMS

//RIGHT SIDE AUTON BLUE AWP
ASSET(R1BLUE_txt);
ASSET(R2BLUE_txt);
ASSET(R3BLUE_txt);
ASSET(R4BLUE_txt);
ASSET(R5BLUE_txt);
ASSET(R6BLUE_txt);
ASSET(R7BLUE_txt);
ASSET(R8BLUE_txt);
ASSET(R9BLUE_txt);

//RIGHT SIDE AUTON BLUE ELIMS

//LEFT SIDE AUTON RED AWP
ASSET(L1RED_txt);
ASSET(L2RED_txt);
ASSET(L3RED_txt);
ASSET(L4RED_txt);
ASSET(L5RED_txt);
ASSET(L6RED_txt);
ASSET(L7RED_txt);
ASSET(L8RED_txt);
ASSET(L9RED_txt);
ASSET(L10RED_txt);



//LEFT SIDE AUTON RED ELIMS


//LEFT SIDE AUTON BLUE AWP
ASSET(L1BLUE_txt);
ASSET(L2BLUE_txt);
ASSET(L3BLUE_txt);

//LEFT SIDE AUTON BLUE ELIMS


//AUTON SKILLS





//RIGHT SIDE AUTON//

// RED Alliance Right Side and gets the auton win point
void RED_Right_side_awp() {
	ExpansionClamp.set_value(HIGH);
	pros::delay(500);
	chassis.setPose(-54.984, -35.014, 270);
	chassis.follow(R1RED_txt, 15, 3000, false);
	pros::delay(2000);


	ExpansionClamp.set_value(LOW);
	Intake.move_velocity(600);
	Conveyor.move_velocity(-100);
		


	pros::delay(1000);
	chassis.turnToHeading(180, 1000);
	chassis.follow(R2RED_txt, 15, 3000);
	pros::delay(500);

	chassis.turnToHeading(0, 1000);
	chassis.follow(R3RED_txt, 15, 3000);
	pros::delay(1000);
	Intake.move_velocity(0);
	pros::delay(1000);
}

// RED Alliance Right Side for elimination rounds
void RED_Right_side_elims() {
}

// Blue Alliance Right Side and gets the auton win point
void BLUE_Right_side_awp() {
   	ExpansionClamp.set_value(HIGH);
	pros::delay(500);
	chassis.setPose(53.715, 40.64, 90);

	chassis.follow(R1BLUE_txt, 15, 1500, false);
	pros::delay(2000);
	ExpansionClamp.set_value(LOW);
	pros::delay(500);
	chassis.turnToHeading(0, 1000);
	Conveyor.move_velocity(-100);
	Intake.move_velocity(600);

	chassis.follow(R2BLUE_txt, 15, 1500);

	chassis.follow(R3BLUE_txt, 15, 1500, false);
	chassis.turnToHeading(290, 1000);

	chassis.follow(R4BLUE_txt, 15, 1500);

	chassis.follow(R5BLUE_txt, 15, 2000, false);
	chassis.turnToHeading(275, 1000);
	
	chassis.follow(R6BLUE_txt, 15, 1500);
	chassis.turnToHeading(270, 1000);
	pros::delay(500);

	chassis.follow(R7BLUE_txt, 15, 1500, false);
	chassis.turnToHeading(90, 1000);

	chassis.follow(R8BLUE_txt, 15, 1500);
	//chassis.turnToHeading(90, 1000);

	chassis.follow(R9BLUE_txt, 15, 1500, false);
	pros::delay(2000);
}

// Blue Alliance Right Side elimination rounds
void BLUE_Right_side_elims() {
}


//LEFT SIDE AUTON//

// RED Alliance Left Side and gets the auton win point
//still testing not final
void RED_LEFT_side_awp() {
    ExpansionClamp.set_value(HIGH);
	pros::delay(500);
	chassis.setPose(-53.715, 40.64, 270);

	chassis.follow(L1RED_txt, 15, 1500, false);
	pros::delay(2000);
	ExpansionClamp.set_value(LOW);
	pros::delay(500);
	chassis.turnToHeading(0, 1000);
	Conveyor.move_velocity(-100);
	Intake.move_velocity(600);

	chassis.follow(L2RED_txt, 15, 1500);

	chassis.follow(L3RED_txt, 15, 1500, false);
	chassis.turnToHeading(70, 1000);

	chassis.follow(L4RED_txt, 15, 1500);

	chassis.follow(L5RED_txt, 15, 2000, false);
	chassis.turnToHeading(85, 1000);
	
	chassis.follow(L6RED_txt, 15, 1500);
	chassis.turnToHeading(90, 1000);
	pros::delay(500);

	chassis.follow(L7RED_txt, 15, 1500, false);
	chassis.turnToHeading(270, 1000);

	chassis.follow(L8RED_txt, 15, 1500);
	pros::delay(500);

	chassis.follow(L9RED_txt, 15, 1500, false);
	pros::delay(2000);

}

// RED Alliance Left Side elimination rounds
void RED_LEFT_side_elims() {
}
// Blue Alliance Left Side and gets the auton win point
void BLUE_LEFT_side_awp() {
    ExpansionClamp.set_value(HIGH);
	pros::delay(500);
	chassis.setPose(53.84, -30.107, 90);
	chassis.follow(L1BLUE_txt, 15, 2000, false);
	pros::delay(2000);


	ExpansionClamp.set_value(LOW);
	Intake.move_velocity(600);
	Conveyor.move_velocity(-100);
		


	pros::delay(1000);
	chassis.turnToHeading(180, 1000);
	chassis.follow(L2BLUE_txt, 15, 2000);
	pros::delay(500);

	chassis.turnToHeading(0, 1000);
	chassis.follow(L3BLUE_txt, 15, 2000);
	pros::delay(1000);
	Intake.move_velocity(0);
	pros::delay(1000);
}

// Blue Alliance Left Side for elimination rounds
void BLUE_LEFT_side_elims() {
}


//SKILLS//
void skills() {
}


void on_center_button() {}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    //pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
	/*
	//Uncomment for testing COMMENT FOR TOURNEMENTS
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });*/
}
/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}


/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize()
{
	chassis.calibrate(); // calibrate sensors
	ms::set_autons({  // Vector of categories
        ms::Category("Red", {
            ms::Auton("Right Side AWP", RED_Right_side_awp),
            ms::Auton("Right Side ELIMS", RED_Right_side_elims),
			ms::Auton("Left Side AWP", RED_LEFT_side_awp),
            ms::Auton("Left Side ELIMS", RED_LEFT_side_elims)
        }),
		ms::Category("Blue", {
            ms::Auton("Right Side AWP", BLUE_Right_side_awp),
            ms::Auton("Right Side ELIMS", BLUE_Right_side_elims),
			ms::Auton("Left Side AWP", BLUE_LEFT_side_awp),
            ms::Auton("Left Side ELIMS", BLUE_LEFT_side_elims)
        }),
        ms::Category("Skills", {
            ms::Auton("Skills", skills)
        })
    });
    ms::initialize(); // Initialize the screen

}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

 // path file name is "example.txt".
// "." is replaced with "_" to overcome c++ limitations


void autonomous() 
{
	//robot size is 16x16 inch

	ms::call_selected_auton();

	pros::delay(1000);
	Conveyor.move_velocity(0);
	Intake.move_velocity(0);
	driveR_train.move_voltage(0);
	driveL_train.move_voltage(0);
	printf("done");
	
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol()
{
	int left;
	int right;
	double Hue;
	bool IntakeState;
	bool IntakeREV;
	bool top_speed;

	top_speed = false;
	Color_sensor.set_led_pwm(10);

	pros::Task my_task(LadyBrownTask);

	while(true){
		
		/*TANK CONTROL*/
		/*
		driveR_train.set_reversed(true);
		driveL_train.move(master.get_analog(ANALOG_LEFT_Y));
		driveR_train.move(master.get_analog(ANALOG_RIGHT_Y));
		*/

		/*CURVATURE CONTROL*/
		/*
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		chassis.curvature(leftY, rightX);
		*/


		/*ARCADE CONTROLL*/

		int power = -(master.get_analog(ANALOG_RIGHT_X));
		int turn = master.get_analog(ANALOG_LEFT_Y);
		left = power - turn;
		right = power + turn;

		driveL_train.move(-left);
		driveR_train.move(right);

		//INTAKE CONTROL
		if(master.get_digital_new_press(DIGITAL_A))
		{
			if(IntakeState == true)
			{
				Conveyor.move_velocity(0);
				Intake.move_velocity(0);
				top_speed = false;
				IntakeState = false;
			}
			else
			{
				Conveyor.move_velocity(200);
				Intake.move_velocity(-200);
				IntakeState = true;
			}
			printf("Intake state=%d top_speed=%d intake velocity=%f \n", IntakeState, top_speed, Intake.get_actual_velocity());
		}

		//REVERSE INTAKE
		if(master.get_digital_new_press(DIGITAL_B))
		{
			if(IntakeREV == true)
			{
				Conveyor.move_velocity(0);
				Intake.move_velocity(0);
				IntakeREV = false;
			}
			else
			{
				Conveyor.move_velocity(-200);
				Intake.move_velocity(200);
				IntakeREV = true;
			}
			top_speed = false;
			IntakeState = false;
			printf("Intake state=%d top_speed=%d intake velocity=%f \n", IntakeState, top_speed, Intake.get_actual_velocity());
			pros::delay(100);
		}


		//COLOR SORT BLUE ALLIANCE
		/*Hue = Color_sensor.get_hue(); 
		if(Hue < 5.0 and IntakeState == true)
		{
			pros::c::delay(175);
			Conveyor.move_voltage(0);
			Intake.move_velocity(0);
			IntakeState = false;
			top_speed = false;
			printf("Color Hue=%f Current Hue=%f \n", Hue, Color_sensor.get_hue());
		}*/

		//LADYBROWN ARM
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
		{
			LadyBrownArm(48500, 2000);
			LadyBrownState = true;
		}

		//LADYBROWN HOLD
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
		{
			LadyBrownArm(56000, 2000); 
			LadyBrownState = false;
		}

		//LADYBROWN SCORE
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
		{
			LadyBrownArm(111000, 2000); 
			LadyBrownState = false;
		}

		//LADYBROWN STOW
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
		{
			LadyBrownArm(31000, 2000); 
			LadyBrownState = false;
		}


		//MOGO CLAMP
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1))
		{
			if(ExpansionClampState == true)
			{

				ExpansionClamp.set_value(LOW);
				ExpansionClampState = false;
			}
			else
			{
				ExpansionClamp.set_value(HIGH);
				ExpansionClampState = true;
			}
			printf("Expansion state=%d \n", ExpansionClampState);
		}
		

		//INTAKE EXPANSION
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))
		{
			if(ExpansionIntakeState == true)
			{
				ExpansionIntake.set_value(LOW);
				ExpansionIntakeState = false;
			}
			else
			{
				ExpansionIntake.set_value(HIGH);
				ExpansionIntakeState = true;
			}
			printf("Expansion state=%d \n", ExpansionIntakeState);
		}

		//DOINKER
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2))
		{
			if(DoinkerState == true)
			{
				Doinker.set_value(LOW);
				DoinkerState = false;
			}
			else
			{
				Doinker.set_value(HIGH);
				DoinkerState = true;
			}
			printf("Expansion state=%d \n", ExpansionIntakeState);
		}

		//UNSTUCK CONVEYOR
		if(Conveyor.get_actual_velocity() < 5 and top_speed == true)
		{
			if(LadyBrownState == true)
			{
				pros::delay(300);
				Conveyor.move_velocity(0);
			}
			else
			{
				Conveyor.move_velocity(-200);
				pros::c::delay(300);
				Conveyor.move_velocity(200);
				top_speed = false;
			}
		}

		//CHECK SPEED
		if(Conveyor.get_actual_velocity() >= 180 and IntakeState == true)
		{
			top_speed = true;
			printf("top_speed=%d \n", top_speed);
		}
		
		//printf("angle=%d, postition=%d \n", LadyBrownRotate.get_angle(), LadyBrownRotate.get_position());
		//printf("hue=%f \n", Color_sensor.get_hue());

		pros::delay(10);
	};
}
