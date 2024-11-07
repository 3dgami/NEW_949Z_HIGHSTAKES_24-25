#include "main.h"
//#include "selection.h"
//#include "selection.ccp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/motors.h"



//update all motor ports if needed
pros::Controller master{CONTROLLER_MASTER};	
pros::Motor right_front(10, pros::E_MOTOR_GEAR_600);
pros::Motor left_front(12, pros::E_MOTOR_GEAR_600);
pros::Motor left_back(11, pros::E_MOTOR_GEAR_600);
pros::Motor right_back(1, pros::E_MOTOR_GEAR_600);
pros::Motor right_mid(33, pros::E_MOTOR_GEAR_600);
pros::Motor left_mid(22, pros::E_MOTOR_GEAR_600);
pros::Motor Conveyor(19);
pros::Motor Intake(2);
pros::Optical Color_sensor(5);
pros::Distance Distance_sensor(3);
pros::Distance Distance_clamp(4);
pros::MotorGroup driveL_train({11, 12});//UPDATE WITH MOTOR WIRING CHANGING
pros::MotorGroup driveR_train({10, 1});
pros::MotorGroup full_drivetrain({11, 12, 10, 1});
pros::IMU imu(3);
//pros::IMU imu2(9);

pros::MotorGroup driveL_trainLem({-11, -12});//UPDATE WITH MOTOR WIRING CHANGING
pros::MotorGroup driveR_trainLem({10, 1});

// drivetrain settings
lemlib::Drivetrain drivetrain(&driveL_trainLem, // left motor group
                              &driveR_trainLem, // right motor group
                              14, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
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
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
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

void on_center_button() {}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
// initialize function. Runs on program startup
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });

	imu.reset(true);
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
{}

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
ASSET(Bottom_path_pt1_txt);
ASSET(example_txt);
ASSET(pathjerryio_txt);
ASSET(TEST1_txt);
ASSET(TEST2_txt);
ASSET(TEST3_txt);
ASSET(TEST4_txt);

void autonomous() 
{
	bool ExpansionClampState;
	bool ExpansionIntakeState;
	auto ExpansionClamp = 'A';
	auto ExpansionIntake ='D';

	pros::c::adi_pin_mode(ExpansionIntake, OUTPUT);
	pros::c::adi_digital_write(ExpansionIntake, LOW);

	pros::c::adi_pin_mode(ExpansionClamp, OUTPUT);
	pros::c::adi_digital_write(ExpansionClamp, LOW);
	//1350 one tile

   // chassis.setPose(-54.465, -23.905, 0);
	//chassis.follow(pathjerryio_txt, 10, 6000);
	//chassis.setPose(-61.28, -9.329, 90);
	//chassis.setPose(-61.469, -9.139, 90);
	//chassis.follow(TEST2_txt, 10, 12000);
	//chassis.follow(TEST3_txt, 10, 100000);
	//chassis.moveToPose(-61.469, -9.139, 90, 10000);
	chassis.setPose(-71.88, -9.888, 90);
	chassis.follow(TEST4_txt, 15, 10000);
	//chassis.follow(example_txt, 15, 4000);
    // move 48" forwards //24 one square
    //chassis.moveToPoint(0, 24, 10000);
	//chassis.turnToHeading(270, 4000);
	/*driveTrain(1350/2, 1000);
	gyroTurn(270);
	Conveyor.move_velocity(100);
	pros::delay(500);
	gyroTurn(180);
	Intake.move_velocity(600);
	Conveyor.move_velocity(0);
	driveTrain(1350/2, 1000);
	gyroTurn(225);
	pros::c::adi_digital_write(ExpansionClamp, HIGH);
	driveTrain(1909, 1000);
	pros::c::adi_digital_write(ExpansionClamp, LOW);
	gyroTurn(45);
	driveTrain(1350, 1000);
	gyroTurn(45);
	driveTrain(1350*1.5, 1000);
	gyroTurn(160);
	Intake.move_velocity(0);
	driveTrain(1350*3, 2000);
	gyroTurn(335);*/

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
	bool ExpansionClampState;
	bool ExpansionNeutral;
	bool ExpansionIntakeState;
	bool DoinkerState;
	bool top_speed;
	top_speed = false;
	auto ExpansionClamp = 'A';
	auto ExpansionIntake ='D';
	auto NeutralStake ='C';
	auto NeutralStake2 ='B';
	auto Doinker ='E';

	pros::c::adi_pin_mode(ExpansionIntake, OUTPUT);
	pros::c::adi_digital_write(ExpansionIntake, LOW);

	pros::c::adi_pin_mode(ExpansionClamp, OUTPUT);
	pros::c::adi_digital_write(ExpansionClamp, LOW);

	pros::c::adi_pin_mode(NeutralStake, OUTPUT);
	pros::c::adi_digital_write(NeutralStake, LOW);

	pros::c::adi_pin_mode(NeutralStake2, OUTPUT);
	pros::c::adi_digital_write(NeutralStake2, LOW);

	pros::c::adi_pin_mode(Doinker, OUTPUT);
	pros::c::adi_digital_write(Doinker, LOW);

	//pros::c::adi_digital_write(ExpansionClamp, HIGH);

	while(true){
		
		/*TANK CONTROL*/
		/*
		driveR_train.set_reversed(true);
		driveL_train.move(master.get_analog(ANALOG_LEFT_Y));
		driveR_train.move(master.get_analog(ANALOG_RIGHT_Y));
		*/

		/*CURVATURE CONTROL*/
		// get left y and right x positions
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
				Conveyor.move_velocity(-100);
				Intake.move_velocity(600);
				IntakeState = true;
			}
			printf("Intake state=%d top_speed=%d intake velocity=%f \n", IntakeState, top_speed, Intake.get_actual_velocity());
		}

		if(master.get_digital_new_press(DIGITAL_B))
		{
			if(IntakeState == true)
			{
				Conveyor.move_velocity(-300);
				
			}
			printf("Intake state=%d intake velocity=%f \n", IntakeState, Intake.get_actual_velocity());
		}


		//COLOR SORT BLUE ALLIANCE
		Hue = Color_sensor.get_hue(); 
		if(Hue == 0 and IntakeState == true)
		{
			Conveyor.move_velocity(-300);
			pros::c::delay(300);
			Conveyor.move_velocity(-100);
			printf("Color Hue=%f Current Hue=%f \n", Hue, Color_sensor.get_hue());
		}

		//AUTO CLAMP
		if(Distance_clamp.get() <= 20 and ExpansionClampState == false)
		{
			pros::c::adi_digital_write(ExpansionClamp, HIGH);
			ExpansionClampState = true;
			printf("Distance to mogo=%d \n", Distance_clamp.get());
		}

		//MOGO CLAMP
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2))
		{
			if(ExpansionClampState == true)
			{
				pros::c::adi_digital_write(ExpansionClamp, LOW);
				ExpansionClampState = false;
			}
			else
			{
				pros::c::adi_digital_write(ExpansionClamp, HIGH);
				ExpansionClampState = true;
			}
			printf("Expansion state=%d \n", ExpansionClampState);
		}
		

		//NUETRAL STAKE EXPANSION
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2))
		{
			if(ExpansionNeutral == true)
			{
				pros::c::adi_digital_write(NeutralStake, LOW);
				pros::c::adi_digital_write(NeutralStake2, LOW);
				ExpansionNeutral = false;
				
			}
			else
			{
				pros::c::adi_digital_write(NeutralStake, HIGH);
				pros::c::adi_digital_write(NeutralStake2, HIGH);
				ExpansionNeutral = true;
			}
			printf("Expansion state=%d \n", ExpansionNeutral);
		}


		//INTAKE EXPANSION
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))
		{
			if(ExpansionIntakeState == true)
			{
				pros::c::adi_digital_write(ExpansionIntake, LOW);
				ExpansionIntakeState = false;
			}
			else
			{
				pros::c::adi_digital_write(ExpansionIntake, HIGH);
				ExpansionIntakeState = true;
			}
			printf("Expansion state=%d \n", ExpansionIntakeState);
		}

		//DOINKER
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1))
		{
			if(DoinkerState == true)
			{
				pros::c::adi_digital_write(Doinker, LOW);
				DoinkerState = false;
			}
			else
			{
				pros::c::adi_digital_write(Doinker, HIGH);
				DoinkerState = true;
			}
			printf("Expansion state=%d \n", ExpansionIntakeState);
		}

		if(Conveyor.get_actual_velocity() > -10 and top_speed == true)
		{

			Conveyor.move_velocity(100);
			pros::c::delay(300);
			Conveyor.move_velocity(-100);
			top_speed = false;
		}

		if(Conveyor.get_actual_velocity() <= -80 and IntakeState == true)
		{
			top_speed = true;
			printf("top_speed=%d \n", top_speed);
		}

		//printf("top_speed=%d \n", top_speed);

		//int pos = getPos();
		//printf("pos=%d \n", pos);
		//double motor_pos = right_back.get_position();
		//printf("pos=%f \n", motor_pos);

		pros::delay(5);
	};
}
