#include "main.h"
#include "config.h"
#include "lemlib/api.hpp"

class MockIMU : public pros::Imu

{
public:
	MockIMU(int port, double gain)
		: pros::Imu(port), imu_gain(gain) {}

	double get_rotation() const override
	{
		double raw = pros::Imu::get_rotation();
		if (raw == PROS_ERR_F)
			return NAN;
		return raw * imu_gain;
	}

private:
	double imu_gain;
};

MockIMU imu(IMU, 361.5 / 360.0);
pros::Controller controller(pros::E_CONTROLLER_MASTER);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button()
{
	static bool pressed = false;
	pressed = !pressed;
	if (pressed)
	{
		pros::lcd::set_text(2, "I was pressed!");
	}
	else
	{
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
lemlib::Drivetrain drivetrain(&left_mg,					  // left motor group
							  &right_mg,				  // right motor group
							  11,						  // 12 inch track width
							  lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
							  450,						  // drivetrain rpm is 450
							  2							  // horizontal drift is 2 (for now)
);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_odom, lemlib::Omniwheel::NEW_2, 0.5);
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
							nullptr,				  // vertical tracking wheel 2, set to nullptr as we are using IMEs
							nullptr,				  // horizontal tracking wheel 1
							nullptr,				  // horizontal tracking wheel 2, set to nullptr as we don't have a second one
							&imu					  // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(9,   // proportional gain (kP)
											  0,   // integral gain (kI)
											  20,  // derivative gain (kD)
											  0,   // anti windup
											  0,   // small error range, in inches
											  100, // small error range timeout, in milliseconds
											  3,   // large error range, in inches
											  500, // large error range timeout, in milliseconds
											  20   // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(3.2, // proportional gain (kP)
											  0,   // integral gain (kI)
											  28,  // derivative gain (kD)
											  0,   // anti windup
											  0,   // small error range, in degrees
											  0,   // small error range timeout, in milliseconds
											  0,   // large error range, in degrees
											  0,   // large error range timeout, in milliseconds
											  0	   // maximum acceleration (slew)
);

lemlib::ExpoDriveCurve
	steer_curve(3,	  // joystick deadband out of 127
				10,	  // minimum output where drivetrain will move out of 127
				1.019 // expo curve gain0
	);

lemlib::ExpoDriveCurve
	throttle_curve(3,	 // joystick deadband out of 127
				   10,	 // minimum output where drivetrain will move out of 127
				   1.019 // expo curve gain
	);

lemlib::Chassis chassis(drivetrain,			// drivetrain settings
						lateral_controller, // lateral PID settings
						angular_controller, // angular PID settings
						sensors				// odometry sensors
											// &throttle_curve, &steer_curve
);



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
//variables
double WheelPos;
double target = 0;
double toutput = 0;
bool holdEnabled = true;
pros::Task *holdTask = nullptr;
double holdTarget = 0;
bool scoring = false;
void frontWheelHoldTask(void *)
{
	const double tkP = 2.0;
	const double tkI = 0;
	const double tkD = 2.0;

	double terror = 0;
	double tprevious_error = 0;
	double tintegral = 0;
	double tderivative = 0;

	while (true)
	{
		if (!holdEnabled)
		{
			LastWheel.move(0);
			pros::delay(20);
			continue;
		}

		// Use the *global* holdTarget, not a local variable
		double currentPos = LastWheel.get_position() / 100.0;
		terror = holdTarget - currentPos;
		tintegral += terror;
		tderivative = terror - tprevious_error;

		double toutput = tkP * terror + tkI * tintegral + tkD * tderivative;
		LastWheel.move(toutput);

		tprevious_error = terror;
		pros::delay(20);
	}
}

void initialize()
{
	pros::lcd::initialize();
	holdTask = new pros::Task(frontWheelHoldTask, nullptr, "Hold Task");
	holdTarget = LastWheel.get_position() / 100.0;

}

void competition_initialize() {}

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
void autonomous() {}

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
// bools
bool IntakeToggle = false;
bool IntakeReverse = false;
bool IntakeEject = false;
bool liftpress = false;
bool scraper = false;
bool tounguepress = false;
void opcontrol()
{
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	while (true)
	{
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
						 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
						 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0); // Prints status of the emulated screen LCDs

		// Arcade control scheme
		int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		// move the robot
		chassis.arcade(leftY, rightX*0.8);
		// index
		//  Indexing (R1)
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
		{
			holdEnabled = false; // disable hold while moving
			Intake_index_mg.move(-127);
			liftpress = false;
			lift.set_value(false);
		}
		// Scoring (L1)
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
		{
			holdEnabled = false;
			Intake_mg.move(-127);
		}
		// Reverse intake (R2)
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
		{
			holdEnabled = false;
			Intake_mg.move(127);
		}
		// Nothing pressed — stop and re-enable hold
		else
		{
			Intake_index_mg.move(0);
			Intake_mg.move(0);
			Intake_eject_mg.move(0);

			// If we just released a button, capture the current position as new hold target
			if (!holdEnabled)
			{
				holdTarget = LastWheel.get_position() / 100.0;
				holdEnabled = true;
			}
		}

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
			liftpress = !liftpress;
			if (liftpress){

			lift.set_value(true);	

			}
			else{

			lift.set_value(false);
			}
		}

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
			tounguepress = !tounguepress;
			if (tounguepress)
			{

				Toungue.set_value(true);
			}
			else
			{

				Toungue.set_value(false);
			}
		}
	}
		// if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
		// {
		// 	scraper = !scraper;
		// 	if (scraper)
		// 	{

		// 		scrape.set_value(true);
		// 	}
		// 	else
		// 	{

		// 		scrape.set_value(false);
		// 	}
		// }

	// Sets right motor voltage
	pros::delay(25); // Run for 20 ms then update
	}


