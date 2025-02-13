#include "main.h"
//#include "autoSelect/selection.h"
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

enum Signatures

{

    Blue = 1,

    Red = 2,

};

pros::Vision vision_sensor{15, pros::E_VISION_ZERO_CENTER};

pros::vision_signature_s_t BLUE_SIG = pros::c::vision_signature_from_utility(Signatures::Blue, 8213, 10745, 9479, -989, 87, -451, 3.000, 0);

pros::vision_signature_s_t RED_SIG = pros::c::vision_signature_from_utility(Signatures::Red, -4199, -3455, -3827, 4681, 9059, 3471, 3.000, 0);


pros::Controller master{CONTROLLER_MASTER};	
pros::Motor right_front(11,pros::E_MOTOR_GEAR_600);
pros::Motor left_front(-20, pros::E_MOTOR_GEAR_600);
pros::Motor left_back(-12,pros::E_MOTOR_GEAR_600);
pros::Motor right_back(19,pros::E_MOTOR_GEAR_600);
pros::Motor right_mid(14,pros::E_MOTOR_GEAR_600);
pros::Motor left_mid(-13,pros::E_MOTOR_GEAR_600);
pros::MotorGroup driveL_train({left_front, left_mid, left_back});//UPDATE WITH MOTOR WIRING CHANGING
pros::MotorGroup driveR_train({right_front, right_mid, right_back});

pros::Motor IntakeConveyor(18);
pros::Motor Intake(4);
pros::Motor LadyBrownLeft(9);
pros::Motor LadyBrownRight(1);
pros::Rotation LadyBrownRotate(3);
pros::Optical Color_sensor(19);
pros::IMU imu(10);

bool ExpansionClampState;
bool DoinkerState;
bool IntakeLiftState = false;
bool LadyBrownState;
bool AllianceBlue;
int position = 15000;
double Hue;

pros::ADIDigitalOut ExpansionClamp('A');
pros::ADIDigitalOut Doinker('B');
pros::ADIDigitalOut IntakeLift('H');

// drivetrain settings //UPDATE
lemlib::Drivetrain drivetrain(&driveL_train, // left motor group
                              &driveR_train, // right motor group
                              13, // 13 inch track width
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
                            nullptr, // horizontal tracking wheel
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

		printf("O=%0.2f, P=%0.2f, D=%0.2f, Position=%d, Err=%d, rotationPOs=%d \n",output, P, D, Pos, errorTerm, LadyBrownRotate.get_position());

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
	Color_sensor.set_led_pwm(50);

	if(AllianceBlue == true)
	{
		while(true)
		{
			if(Hue < 15.0 and IntakeConveyor.get_target_velocity() == 600)
			{
			pros::c::delay(250);
			IntakeConveyor.move_velocity(0);
			pros::c::delay(250);
			IntakeConveyor.move_velocity(600);
			printf("Color Hue=%f Current Hue=%f \n", Hue, Color_sensor.get_hue());
			}
			pros::delay(5);
		}
	}
	else 
	{
		while(true)
		{
			if(220 < Hue and Hue > 260 and IntakeConveyor.get_target_velocity() == 600) 
			{
			pros::c::delay(250);
			IntakeConveyor.move_velocity(0);
			pros::c::delay(250);
			IntakeConveyor.move_velocity(600);
			printf("Color Hue=%f Current Hue=%f \n", Hue, Color_sensor.get_hue());
			}
			pros::delay(5);
		}
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


void MoveVisionAssisted(int timeOut)
{	

	int speed;
	vision_sensor.set_signature(Signatures::Blue, &BLUE_SIG);

    vision_sensor.set_signature(Signatures::Red, &RED_SIG);

    vision_sensor.set_exposure(50);
	pros::vision_object_s_t obj;
    while(timeOut > 0)
	{
		speed = 45;
		double offset;
		SetDrive(speed/2 , speed/2);
		if (vision_sensor.read_by_size(0, 1, &obj) == 1 && obj.top_coord + obj.height > 0)
            {
				// y <= 100
				// w == 200 - too close, done
				// x <0 - turn left
				offset = 1 + 0.5 * abs(obj.x_middle_coord) / obj.width;
				if(offset > 1.6)
				{
					offset = 1.6;
				}
		
				if(obj.x_middle_coord < -5)
				{
					SetDrive(speed, speed * offset);
				} else if(obj.x_middle_coord > 5)
				{
					SetDrive(speed * offset, speed);
				} else {
					SetDrive(speed, speed);
				}
				if(obj.width > 180)
				{
					break;
				}

                // Positive offset means goal is left due to sensor being mounted upside down
                printf("w:%d  x:%d  y:%d  offset:%f\n", obj.width, obj.x_middle_coord, obj.y_middle_coord, offset);

            }
		pros::delay(10);
		timeOut -= 10;
	}
	SetDrive(0,0);
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
	pros::delay(750);
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
	chassis.turnToHeading(0, 1000);
	chassis.follow(R5RED_txt, 15, 2500);
}

// RED Alliance Right Side for elimination rounds
void RED_Right_side_elims() {
	AllianceBlue = false;
	AllianceBlue = false;
	chassis.setPose(-52.311, -58.847, 270);
	chassis.follow(R1RED_txt, 15, 3500, false);
	pros::delay(2000);
	ExpansionClamp.set_value(true);
	pros::delay(750);
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
	chassis.setPose(-51.863, 23.217, 270);
	chassis.follow(L1RED_txt, 15, 2000, false);
	pros::delay(750);
	ExpansionClamp.set_value(true);
	pros::delay(750);
	Intake.move_velocity(200);
	pros::delay(750);
	chassis.turnToHeading(0, 1500);
	chassis.follow(L2RED_txt, 15, 2000);
	pros::delay(750);
	//chassis.turnToHeading(300, 1500);
	chassis.turnToHeading(85, 1500);
	chassis.follow(L3RED_txt, 15, 2000);
	chassis.follow(L4RED_txt, 15, 2000, false);
	chassis.turnToHeading(180, 1500);
	chassis.follow(L5RED_txt, 15, 2000);


}

// RED Alliance Left Side elimination rounds
void RED_LEFT_side_elims() {
	AllianceBlue = false;
}
// Blue Alliance Left Side and gets the auton win point
void BLUE_LEFT_side_awp() {
	AllianceBlue = true;
	chassis.setPose(52.311, -58.847, 90);
	chassis.follow(L1BLUE_txt, 15, 3500, false);
	pros::delay(2000);
	ExpansionClamp.set_value(true);
	pros::delay(750);
	IntakeConveyor.move_velocity(600);
	chassis.follow(L2BLUE_txt, 15, 6000);
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
	chassis.turnToHeading(0, 1000);
	chassis.follow(L5BLUE_txt, 15, 2500);
	//chassis.follow(L6BLUE_txt, 15, 2500, false);
	//pros::delay(2000);
	//chassis.turnToHeading(0, 1000);
	//chassis.follow(L7BLUE_txt, 15, 2500);


}

// Blue Alliance Left Side for elimination rounds
void BLUE_LEFT_side_elims() {
	AllianceBlue = true;
	chassis.setPose(52.311, -58.847, 90);
	chassis.follow(L1BLUE_txt, 15, 3500, false);
	pros::delay(2000);
	ExpansionClamp.set_value(true);
	pros::delay(750);
	IntakeConveyor.move_velocity(600);
	chassis.follow(L2BLUE_txt, 15, 6000);
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
	chassis.setPose(-58.809, -0.707, 90);
	IntakeConveyor.move_velocity(600);
	Intake.move_velocity(-600);
	pros::delay(1000);
	chassis.follow(Skills1_txt, 15, 3000);
	chassis.turnToHeading(0, 1000);
	pros::delay(1000);
	chassis.follow(Skills2_txt, 15, 3000, false);
	pros::delay(500);
	ExpansionClamp.set_value(true);
	pros::delay(1000);
	chassis.turnToHeading(90, 1000);
	MoveVisionAssisted(5000);
	chassis.turnToHeading(180, 1000);
	MoveVisionAssisted(5000);
	chassis.turnToHeading(90, 1000);
	chassis.setPose(0, 0, 90);
	chassis.follow(Skills3_txt, 15, 3000);
	MoveVisionAssisted(5000);
	chassis.turnToHeading(270, 1000);
	chassis.setPose(0, 0, 270);
	chassis.follow(Skills4_txt, 15, 3000);
	MoveVisionAssisted(5000);

	
	//chassis.follow(Skills10_txt, 15, 3000, false);

	
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
	pros::Task LadyBrownTask{[=] {
        while(true)
		{
		LadyBrownArm();
		}
    }
	};
	//pros::c::task_create(ColorSort, (void*)("") , TASK_STACK_DEPTH_DEFAULT, TASK_STACK_DEPTH_MIN, "ColorSort");
	//pros::Task ColorSortTask(ColorSort);
	/*
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
	bool IntakeState = false;
	bool IntakeREV = false;
	bool top_speed = false;

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
				Intake.move_velocity(-600);
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
				Intake.move_velocity(600);
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
			position = 15000;
		}

		//HOLD
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
		{
			position = -14000;
		}

		//ARM
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
		{
			position = 1500;
		}

		//UNSTUCK CONVEYOR
		/*if(IntakeConveyor.get_actual_velocity() > 5 and top_speed == true and LadyBrownState == true)
		{
			IntakeConveyor.move_velocity(0);
			top_speed = false;
		}

		//CHECK SPEED
		if(IntakeConveyor.get_actual_velocity() >= 180 and IntakeState == true)
		{
			top_speed = true;
			printf("top_speed=%d \n", top_speed);
		}*/
		
		
		//printf("angle=%d, postition=%d \n", LadyBrownRotate.get_angle(), LadyBrownRotate.get_position());
		//printf("hue=%f \n", Color_sensor.get_hue());

		pros::delay(10);
	};
}
