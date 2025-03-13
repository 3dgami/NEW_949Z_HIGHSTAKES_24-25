#include "main.h"
//#include "autoSelect/selection.h"
#include "Master-Selector/selector.hpp"
#include "pros/vision.hpp"
#include "pros/vision.h"
#include "Master-Selector/api.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <cstdio>

enum Signatures
{
	First = 1,
    Blue = First,
    Red = 2,
	Yellow = 3,
	Empty = 4,
};

pros::Vision vision_sensor{15, pros::E_VISION_ZERO_CENTER};
pros::vision_signature_s_t BLUE_SIG = pros::c::vision_signature_from_utility(Signatures::Blue, 8213, 10745, 9479, -989, 87, -451, 4.000, 0);
pros::vision_signature_s_t RED_SIG = pros::c::vision_signature_from_utility(Signatures::Red, -4199, -3455, -3827, 4681, 9059, 3471, 5.000, 0);
pros::vision_signature_s_t YELLOW_SIG = pros::c::vision_signature_from_utility(Signatures::Yellow, -2239, -1723, -1980, -5487, -4611, -5048, 6.00, 0);
pros::vision_signature_s_t YELLOW_EMPTY = pros::c::vision_signature_from_utility(Signatures::Empty, 8213, 10745, 9479, -989, 87, -451, 0.1, 0);

pros::Controller master{CONTROLLER_MASTER};	
pros::Motor right_front(11,pros::E_MOTOR_GEAR_600);
pros::Motor left_front(-20, pros::E_MOTOR_GEAR_600);
pros::Motor left_back(-12,pros::E_MOTOR_GEAR_600);
pros::Motor right_back(19,pros::E_MOTOR_GEAR_600);
pros::Motor right_mid(14,pros::E_MOTOR_GEAR_600);
pros::Motor left_mid(-13,pros::E_MOTOR_GEAR_600);
pros::MotorGroup driveL_train({left_front, left_mid, left_back});//UPDATE WITH MOTOR WIRING CHANGING
pros::MotorGroup driveR_train({right_front, right_mid, right_back});

pros::Motor IntakeConveyor(18, pros::E_MOTOR_GEAR_600);
pros::Motor Intake(-9);
pros::Motor LadyBrownLeft(90);
pros::Motor LadyBrownRight(1);
pros::Rotation LadyBrownRotate(16);
pros::Optical Color_sensor(3);
pros::IMU imu(10);

bool ExpansionClampState;
bool DoinkerState;
bool IntakeLiftState = false;
bool LadyBrownState;
bool AllianceBlue;
bool top_speed = false;
bool IntakeState = false;
int position = 15000;
double Hue;

pros::ADIDigitalOut ExpansionClamp('A');
pros::ADIDigitalOut Doinker('B');
pros::ADIDigitalOut IntakeLift('H');

// drivetrain settings //UPDATE
lemlib::Drivetrain drivetrain(&driveL_train, // left motor group
                              &driveR_train, // right motor group
                              10.5, // 10.5 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 2,75" omnis
                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings lateral_controller(18, // proportional gain (kP)
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
lemlib::ControllerSettings angular_controller(1.5, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             11, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr,// horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     40, // minimum output where drivetrain will move out of 127
                                     1.024 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  40, // minimum output where drivetrain will move out of 127
                                  1.024 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

void LadyBrownArm()
{	

	double kp = 1.2;
	double ki = 1.0;
	double kd = -1.0;   /*derivitive should control and stop overshooting this can be done
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


	while (true) 
	{

		errorTerm = position - LadyBrownRotate.get_position();

		sign = (errorTerm < 0) ? sign = -1 : sign = 1;

		int Pos = LadyBrownRotate.get_angle();

		errorTotal = errorTotal + errorTerm;

		sign = (errorTerm < 0) ? -1 : 1;


		errorTotal = (errorTotal > 500 / ki) ? 500 / ki : errorTotal;

		P = errorTerm * kp;
		//I = errorTotal * ki;
		D = (lastError - errorTerm) * kd;
		double output = ((P + D) + (1 * sign));

		// printf("O=%0.2f, P=%0.2f, D=%0.2f, Position=%d, Err=%d, rotationPOs=%d \n",output, P, D, Pos, errorTerm, LadyBrownRotate.get_position());

		LadyBrownRight.move_voltage(-output);
		LadyBrownLeft.move_voltage(output);
		
		

		lastError = errorTerm;
		pros::delay(10);
		
		//SERT_bool = (errorTerm < 15) ? true : false;
		//SERT_count = (SERT_bool = true) ? SERT_count + 20 : SERT_count;
		//count += 20;

	}
	//LadyBrownRight.move_voltage(0);
	//LadyBrownLeft.move_voltage(0);
	//pros::delay(100);
	printf("End\nErr=%d", errorTerm);

	return;
}

void ColorSort()
{	
	int count;
	int start_count;
	Color_sensor.set_led_pwm(50);

	while(true)
	{
		if(AllianceBlue == true)
		{	
			Hue = Color_sensor.get_hue();
			if(Hue < 15.0) // and IntakeConveyor.get_target_velocity() == 600)
			{
			pros::c::delay(120);
			IntakeConveyor.move_velocity(0);
			pros::c::delay(100);
			IntakeConveyor.move_velocity(600);
			//printf("Color Hue=%f Current Hue=%f \n", Hue, Color_sensor.get_hue());
			}
			//printf("Color Hue=%f Current Hue=%f \n", Hue, Color_sensor.get_hue());
		}
		else 
		{
			Hue = Color_sensor.get_hue();
			if(210 < Hue and Hue < 240) //and IntakeConveyor.get_target_velocity() == 600) 
			{
			pros::c::delay(120);
			IntakeConveyor.move_velocity(0);
			pros::c::delay(100);
			IntakeConveyor.move_velocity(600);
			//printf("Color Hue=%f Current Hue=%f \n", Hue, Color_sensor.get_hue());
			}
			//printf("Color Hue=%f Current Hue=%f \n", Hue, Color_sensor.get_hue());
		}
		pros::delay(5);
	}
}


void SetDrive(int Lspeed, int Rspeed)
{

    left_front.move(Lspeed);
    left_mid.move(Lspeed);
    left_back.move(Lspeed);
    right_front.move(Rspeed);
    right_mid.move(Rspeed);
    right_back.move(Rspeed);
}

void InitVisionSensor() {
	vision_sensor.set_signature(Signatures::Blue, &BLUE_SIG);
    vision_sensor.set_signature(Signatures::Red, &RED_SIG);
    vision_sensor.set_exposure(50);
}

void MoveVisionAssisted(int timeOut)
{
	int maxSize = 0;
	pros::vision_object_s_t obj;

	int speed = 45;

    while(timeOut > 0)
	{
		double offset;
		SetDrive(20, 20);
		if (vision_sensor.read_by_size(0, 1, &obj) == 1 && obj.width > 30)
            {
				if (maxSize < obj.width)
					maxSize = obj.width;

				offset = 1 + abs(obj.x_middle_coord) * 0.003;

				// Positive offset means goal is left due to sensor being mounted upside down
                printf("w:%d  x:%d  y:%d  offset:%f\n", obj.width, obj.x_middle_coord, obj.y_middle_coord, offset);

				if(obj.width > 290 || obj.y_middle_coord > 55) {
					printf("Stop\n");
					break;
				}

				if(offset > 1.4)
				{
					offset = 1.4;
				}
		
				// x < 0 - turn right
				if(obj.x_middle_coord > 5)
				{
					SetDrive(speed / offset, speed * offset);
				} else if(obj.x_middle_coord < -5)
				{
					SetDrive(speed * offset, speed / offset);
				} else {
					SetDrive(speed, speed);
				}
            } else {
				printf(".");
			}
		pros::delay(10);
		timeOut -= 10;
	}

	// let it drive just bit
	SetDrive(30, 30);
	pros::c::delay(750);

	SetDrive(0,0);
}

void MoveVisionAssistedToMogo(int timeOut)
{
	int maxSize = 0;
	pros::vision_object_s_t obj;

	vision_sensor.set_signature(Signatures::First, &YELLOW_SIG);
	vision_sensor.set_signature(Signatures::First + 1, &YELLOW_EMPTY);
	vision_sensor.set_exposure(20);

	int speed = 30;

	SetDrive(20, 20);
    while(timeOut > 0)
	{
		double offset;
		if (vision_sensor.read_by_size(0, 1, &obj) == 1 && obj.width > 30)
            {
				if (maxSize < obj.width)
					maxSize = obj.width;

				offset = 1 + abs(obj.x_middle_coord) * 0.005;

				// Positive offset means goal is left due to sensor being mounted upside down
                printf("w:%d  x:%d  y:%d  offset:%f\n", obj.width, obj.x_middle_coord, obj.y_middle_coord, offset);

				if(obj.width > 220 || obj.y_middle_coord > -50) {
					printf("Stop\n");
					break;
				}

				if(offset > 1.4)
				{
					offset = 1.4;
				}
		
				// x < 0 - turn right
				if(obj.x_middle_coord > 5)
				{
					SetDrive(speed / offset, speed * offset);
				} else if(obj.x_middle_coord < -5)
				{
					SetDrive(speed * offset, speed / offset);
				} else {
					SetDrive(speed, speed);
				}
            } else {
				printf(".");
			}
		pros::delay(10);
		timeOut -= 10;
	}

	SetDrive(0,0);
	InitVisionSensor();
}

void stuck()
{
	while(true)
	{
		if(top_speed == true and IntakeConveyor.get_actual_velocity() < 5)
		{
			IntakeConveyor.move_velocity(-600);
			pros::delay(500);
			IntakeConveyor.move_velocity(600);
			pros::delay(250);

		}
		if(IntakeConveyor.get_actual_velocity() >= 450)
		{
			top_speed = true;
			printf("top_speed=%d \n", top_speed);
		}
		printf("speed%f \n", IntakeConveyor.get_actual_velocity());
		pros::delay(5);
		//printf("running \n");
	}
}

//AUTON//

//RIGHT SIDE AUTON RED AWP
ASSET(R1RED_txt);
ASSET(R2RED_txt);
ASSET(R25RED_txt);
ASSET(R3RED_txt);
ASSET(R4RED_txt);
ASSET(R5RED_txt);
ASSET(R6RED_txt);
ASSET(R65RED_txt);
ASSET(R7RED_txt);
ASSET(R8RED_txt);
ASSET(R9RED_txt);
ASSET(R10RED_txt);

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
ASSET(L1BLUENEW_txt);
ASSET(L3BLUENEW_txt);
ASSET(L2BLUE_txt);
ASSET(L3BLUE_txt);
ASSET(L4BLUE_txt);
ASSET(L5BLUE_txt);
ASSET(L6BLUE_txt);
ASSET(L7BLUE_txt);
ASSET(L8BLUE_txt);
ASSET(L9BLUE_txt);
ASSET(L10BLUE_txt);

//LEFT SIDE AUTON BLUE ELIMS


//AUTON SKILLS
ASSET(Skills1_txt);
ASSET(Skills2_txt);
ASSET(Skills3_txt);
ASSET(Skills4_txt);
ASSET(Skills5_txt);
ASSET(Skills6_txt);
ASSET(Skills7_txt);
ASSET(Skills8_txt);
ASSET(Skills9_txt);
ASSET(Skills10_txt);
ASSET(Skills11_txt);






//RIGHT SIDE AUTON//

// RED Alliance Right Side and gets the auton win point
void RED_Right_side_awp() {
	AllianceBlue = false;
	chassis.setPose(-52.311, -58.847, 270);
	chassis.follow(R1RED_txt, 15, 3500, false);
	pros::delay(2000);
	ExpansionClamp.set_value(true);
	pros::delay(250);
	chassis.follow(R2RED_txt, 15, 6000);
	IntakeConveyor.move_velocity(600);
	pros::delay(1500);
	ExpansionClamp.set_value(false);
	pros::delay(500);
	Intake.move_velocity(-600);
	chassis.turnToHeading(270, 1000);
	chassis.follow(R3RED_txt, 15, 2500, false);
	pros::delay(1000);
	ExpansionClamp.set_value(true);
	pros::delay(1000);
	chassis.turnToHeading(180, 1000);
	chassis.follow(R4RED_txt, 15, 2500);
	pros::delay(1500);
	Intake.move_velocity(0);
	//pros::delay(200);
	chassis.turnToHeading(0, 1000);
	chassis.follow(R5RED_txt, 15, 2500);
}

// RED Alliance Right Side for elimination rounds
void RED_Right_side_elims() {
	AllianceBlue = false;
	chassis.setPose(-52.311, -58.847, 270);
	chassis.follow(R1RED_txt, 15, 3500, false);
	pros::delay(2000);
	ExpansionClamp.set_value(true);
	pros::delay(250);
	IntakeConveyor.move_velocity(600);
	chassis.follow(R2RED_txt, 15, 6000);
	pros::delay(1500);
	ExpansionClamp.set_value(false);
	pros::delay(500);
	Intake.move_velocity(-600);
	chassis.turnToHeading(270, 1000);
	chassis.follow(R3RED_txt, 15, 2500, false);
	pros::delay(1000);
	ExpansionClamp.set_value(true);
	pros::delay(1000);
	chassis.turnToHeading(180, 1000);
	chassis.follow(R4RED_txt, 15, 2500);
	pros::delay(1000);
	Intake.move_velocity(0);
	//pros::delay(200);
	Intake.move_velocity(0);
	chassis.turnToHeading(270, 1000);
	//chassis.follow(R5RED_txt, 15, 2500);
}

// Blue Alliance Right Side and gets the auton win point
void BLUE_Right_side_awp() {
	AllianceBlue = true;
	chassis.setPose(51.863, 23.217, 90);
	chassis.follow(R1BLUE_txt, 15, 2000, false);
	pros::delay(750);
	ExpansionClamp.set_value(true);
	pros::delay(750);
	Intake.move_velocity(200);
	pros::delay(750);
	chassis.turnToHeading(0, 1500);
	chassis.follow(R2BLUE_txt, 15, 2000);
	pros::delay(750);
	chassis.turnToHeading(275, 1500);
	chassis.follow(R3BLUE_txt, 15, 2000);
	chassis.follow(R4BLUE_txt, 15, 2000, false);
	chassis.turnToHeading(180, 1500);
	chassis.follow(R5BLUE_txt, 15, 2000);
}

// Blue Alliance Right Side elimination rounds
void BLUE_Right_side_elims() {
	AllianceBlue = true;
}



//LEFT SIDE AUTON//

// RED Alliance Left Side
void RED_LEFT_side_awp() {
	AllianceBlue = false;
	chassis.setPose(-50.011, 23.602, 270);
	chassis.follow(L1RED_txt, 15, 2000, false);
	chassis.waitUntilDone();
	pros::delay(100);
	ExpansionClamp.set_value(true);
	pros::delay(250);
	IntakeConveyor.move_velocity(600);
	Intake.move_velocity(600);
	pros::delay(250);
	chassis.turnToHeading(45, 2000);
	chassis.waitUntilDone();
	chassis.follow(L2RED_txt, 15, 2000, true);
	chassis.waitUntilDone();
	chassis.turnToHeading(0, 1000);
	chassis.waitUntilDone();
	chassis.follow(L3RED_txt, 15, 2000, true);
	chassis.waitUntilDone();
	chassis.follow(L4RED_txt, 15, 3000, false);
	chassis.waitUntilDone();
	chassis.turnToHeading(0, 2000);
	chassis.waitUntilDone();
	chassis.follow(L5RED_txt, 15, 2000, true);
	chassis.waitUntilDone();
	chassis.turnToHeading(180, 2000);
	chassis.waitUntilDone();
	chassis.follow(L6RED_txt, 15, 2000, true);
	


}

// RED Alliance Left Side elimination rounds
void RED_LEFT_side_elims() {
	AllianceBlue = false;
}
// Blue Alliance Left Side and gets the auton win point
void BLUE_LEFT_side_awp() {
	IntakeConveyor.move_velocity(0);
	Intake.move_velocity(0);
	AllianceBlue = true;
	chassis.setPose(49.934, -57.707, 90);
	IntakeConveyor.move_velocity(0);
	chassis.follow(L1BLUE_txt, 15, 5000, false);
	chassis.waitUntilDone();
	pros::delay(100);
	ExpansionClamp.set_value(true);
	pros::delay(250);
	IntakeConveyor.move_velocity(600);

	chassis.follow(L2BLUE_txt, 15, 6000);
	pros::delay(1500);
	ExpansionClamp.set_value(false);
	pros::delay(500);

	Intake.move_velocity(-600);
	chassis.turnToHeading(90, 1000);
	chassis.follow(L3BLUE_txt, 15, 2500, false);
	chassis.waitUntilDone();
	ExpansionClamp.set_value(true);
	Intake.move_velocity(600);

	pros::delay(1000);
	chassis.turnToHeading(180, 1000);
	chassis.follow(L4BLUE_txt, 15, 2500);
	pros::delay(1000);
	chassis.turnToHeading(0, 1000);
	chassis.follow(L5BLUE_txt, 15, 2500);


}

// Blue Alliance Left Side for elimination rounds
void BLUE_LEFT_side_elims() {
	AllianceBlue = true;
	chassis.setPose(52.311, -58.847, 90);
	chassis.follow(L1BLUE_txt, 15, 3500, false);
	pros::delay(2000);
	ExpansionClamp.set_value(true);
	pros::delay(250);
	chassis.follow(L2BLUE_txt, 15, 10000);
	IntakeConveyor.move_velocity(600);
	pros::delay(1500);
	ExpansionClamp.set_value(false);
	pros::delay(500);
	Intake.move_velocity(-600);
	chassis.turnToHeading(90, 1000);
	chassis.follow(L3BLUE_txt, 15, 2500, false);
	pros::delay(1000);
	ExpansionClamp.set_value(true);
	pros::delay(1000);
	chassis.turnToHeading(180, 1000);
	chassis.follow(L4BLUE_txt, 15, 2500);
	pros::delay(1000);
	Intake.move_velocity(0);
	chassis.turnToHeading(90, 1000);
	
	//chassis.follow(L5BLUE_txt, 15, 2500);
}


//SKILLS//
void skills() {
	AllianceBlue = false;
	double angle;
	InitVisionSensor();
	chassis.setPose(-58.5, 0, 90);
	IntakeConveyor.move_velocity(600);
	Intake.move_velocity(-600);
	pros::delay(1000);
	chassis.follow(Skills1_txt, 15, 3000);
	chassis.waitUntilDone();
	chassis.turnToHeading(0, 1000);
	chassis.waitUntilDone();
	chassis.follow(Skills2_txt, 10, 3000, false);
	/*ExpansionClamp.set_value(true);
	pros::delay(500);
	chassis.turnToHeading(90, 1000);
	MoveVisionAssisted(5000);
	pros::delay(500);
	chassis.turnToHeading(180, 1000);
	MoveVisionAssisted(2000);
	pros::delay(500);
	chassis.turnToHeading(90, 2000);
	pros::delay(500);
	chassis.setPose(0, 0, 90);
	chassis.moveToPose(30.283, 0.175, 90, 2500);
	MoveVisionAssisted(5000);
	pros::delay(1000);
	chassis.turnToHeading(270, 1000);
	pros::delay(1000);
	chassis.setPose(0, 0, 270);
	chassis.moveToPose(-50.00, -0.175, 270, 3000);
	pros::delay(500);
	chassis.turnToHeading(270, 1000);
	MoveVisionAssisted(5000);
	MoveVisionAssisted(5000);

	pros::delay(1000);
	chassis.turnToHeading(270, 1000);
	pros::delay(1000);

	chassis.setPose(0, 0, 270);
	chassis.moveToPose(25, 0, 270, 2500, {.forwards = false});
	pros::delay(500);
	chassis.turnToHeading(235, 1000);	
	MoveVisionAssisted(5000);
	pros::delay(500);

	//Remove once fix 6 ring jam issue
	IntakeConveyor.move_velocity(-600);
	pros::delay(300);
	IntakeConveyor.move_velocity(0);


	//chassis.setPose(5, 0, 270);
	pros::delay(500);
	chassis.turnToHeading(225, 1000);
	pros::delay(500);

	chassis.setPose(0, 0, 225);
	chassis.moveToPose(5, 5, 225, 2500, {.forwards = false});

	pros::delay(500);
	chassis.turnToHeading(65, 1000);
	pros::delay(500);

	chassis.setPose(0, 0, 65);
	chassis.moveToPose(-19, -11, 65, 2500, {.forwards = false});
	pros::delay(1500);
	ExpansionClamp.set_value(false);

	pros::delay(500);
	chassis.turnToHeading(65, 1000);
	pros::delay(500);

	chassis.setPose(0, 0, 65);
	chassis.moveToPose(6, 6, 65, 2000);
	chassis.turnToHeading(0, 1000);
	chassis.moveToPose(3, -20, 0, 1000, {.forwards = false});
	pros::delay(2000);

	pros::delay(500);
	chassis.turnToHeading(0, 1000);
	pros::delay(500);

	chassis.setPose(0, 0, 0);
	chassis.moveToPose(0, 60, 0, 2500);
	//chassis.turnToHeading(180, 1000);
	pros::delay(500);
	MoveVisionAssistedToMogo(5000);
	pros::delay(500);

	angle = imu.get_rotation() - 90;

	//chassis.turnToHeading(0, 1000);
	pros::delay(1000);
	chassis.turnToHeading(180 + angle, 1000);
	pros::delay(500);
	//angle = imu.get_heading();
	chassis.setPose(0, 0, 180 + angle);

	chassis.moveToPose(0, 15, 180 + angle, 2500, {.forwards = false});
	pros::delay(2500);
	ExpansionClamp.set_value(true);
	pros::delay(500);
	IntakeConveyor.move_velocity(600);

	//after gettign second mogo
	chassis.turnToHeading(90, 1000);
	MoveVisionAssisted(5000);
	pros::delay(500);
	chassis.turnToHeading(0, 1000);
	MoveVisionAssisted(2000);
	pros::delay(500);
	chassis.turnToHeading(90, 2000);
	pros::delay(500);
	chassis.setPose(0, 0, 90);
	chassis.moveToPose(30.283, 0.175, 90, 2500);
	MoveVisionAssisted(5000);
	pros::delay(500);
	chassis.turnToHeading(270, 1000);
	pros::delay(500);
	chassis.setPose(0, 0, 270);
	chassis.moveToPose(-50.00, -0.175, 270, 3000);
	pros::delay(500);
	chassis.turnToHeading(270, 1000);
	MoveVisionAssisted(5000);
	MoveVisionAssisted(5000);
	chassis.setPose(0, 0, 270);
	chassis.moveToPose(25, 0, 270, 2500, {.forwards = false});
	pros::delay(500);
	chassis.turnToHeading(305, 1000);	
	MoveVisionAssisted(5000);
	pros::delay(500);


	//Remove once fix 6 ring jam issue
	IntakeConveyor.move_velocity(-600);
	pros::delay(300);
	IntakeConveyor.move_velocity(0);


	//chassis.setPose(5, 0, 270);
	chassis.turnToHeading(315, 1000);
	chassis.setPose(0, 0, 315);
	chassis.moveToPose(5, -5, 315, 2500, {.forwards = false});
	chassis.turnToHeading(115, 1000);
	pros::delay(1000);
	chassis.setPose(0, 0, 115);
	chassis.moveToPose(-19, 11, 65, 2500, {.forwards = false});
	pros::delay(1500);
	ExpansionClamp.set_value(false);*/

}

void on_center_button() {}


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
	master.set_text(0,5, "Phillipines 4:13");
    chassis.calibrate(); // calibrate sensors
	//chassis.setPose(-50.011, 23.602, 90);
	Color_sensor.set_led_pwm(50);
	pros::Task LadyBrownTask{[=] {
        while(true)
		{
		LadyBrownArm();
		}
    }
	};

	pros::Task ColorSortTask{[=] {
        ColorSort();
    }
	};
	
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
			//pros::lcd::print(1, "Rotation Sensor: %i", horizontal_encoder.get_position());
			//pros::lcd::print(2, "ADI Vertical: %i", vertical_encoder.get_value());
            pros::delay(20);
        }
    });
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
void autonomous() 
{
	
	ms::call_selected_auton();

	pros::delay(5000);
	Intake.move_velocity(0);
	IntakeConveyor.move_velocity(0);
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
	bool IntakeREV = false;
	bool done = false;

	if (false) {
		AllianceBlue = true;
	}

	while(true){

		int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        chassis.arcade(leftY, rightX);

		//INTAKE CONTROL
		if(master.get_digital_new_press(DIGITAL_A))
		{
			if(IntakeState == true)
			{
				Intake.move_velocity(0);
				IntakeConveyor.move_velocity(0);
				top_speed = false;
				IntakeState = false;
			}
			else
			{
				Intake.move_velocity(600);
				IntakeConveyor.move_velocity(600);
				IntakeState = true;
			}
			printf("Intake state=%d top_speed=%d intake velocity=%f \n", IntakeState, top_speed, Intake.get_actual_velocity());
		}

		//REVERSE INTAKE
		if(master.get_digital_new_press(DIGITAL_B))
		{
			if(IntakeREV == true)
			{
				IntakeConveyor.move_velocity(0);
				Intake.move_velocity(0);
				IntakeREV = false;
			}
			else
			{
				IntakeConveyor.move_velocity(-600);
				Intake.move_velocity(-600);
				IntakeREV = true;
			}
			top_speed = false;
			IntakeState = false;
			printf("Intake state=%d top_speed=%d intake velocity=%f \n", IntakeState, top_speed, Intake.get_actual_velocity());
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
		
		//DOINKER
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))
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
			printf("Expansion state=%d \n", DoinkerState);
		}

        //INTAKE LIFT FOR TESTING
		/*if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2))
		{
			if(IntakeLiftState == true)
			{
				IntakeLift.set_value(LOW);
				IntakeLiftState= false;
			}
			else
			{
				IntakeLift.set_value(HIGH);
				IntakeLiftState = true;
			}
			printf("Expansion state=%d \n", IntakeLiftState);
		}*/


		//LADYBROWN CONTROL

		//SCORE
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
		{
			position = -60000;
		}

		//STOW
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
		{
			position = 18000;
		}

		//HOLD
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
		{
			position = -12000;
		}

		//ARM
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
		{
			position = 3500;
		}
		
		// printf("Alliance=%d", AllianceBlue);
		// printf("Conveyor=%f, Intake=%f \n", IntakeConveyor.get_actual_velocity(), Intake.get_actual_velocity());
		//printf("angle=%d, postition=%d \n", LadyBrownRotate.get_angle(), LadyBrownRotate.get_position());
		printf("angle=%f \n", imu.get_rotation());

		pros::delay(10);
	};
}
