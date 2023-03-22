#include "main.h"

// make controller buttons
ControllerButton intakeIn(ControllerDigital::R2);
ControllerButton intakeOut(ControllerDigital::R1);
ControllerButton slowIntakeIn(ControllerDigital::L1);

ControllerButton fastFlywheel(ControllerDigital::A);
ControllerButton slowFlywheel(ControllerDigital::B);

// make chassis
std::shared_ptr<OdomChassisController> chassis =
	ChassisControllerBuilder()
		.withMotors(
			{12, 14, -16},
			{-13, -15, 17})
		// Green gearset, 4 in wheel diam, 11.5 in wheel track
		.withDimensions(AbstractMotor::gearset::green, {{3.25_in, 11.5_in}, imev5GreenTPR})
		.withOdometry()
		.buildOdometry();

// make path generator | copied from okapi tutorials
std::shared_ptr<AsyncMotionProfileController> profileController =
	AsyncMotionProfileControllerBuilder()
		.withLimits({
			2.87,		// Maximum linear velocity of the Chassis in m/s
			2.0 * 2.87, // Maximum linear acceleration of the Chassis in m/s/s
			10.0 * 2.87 // Maximum linear jerk of the Chassis in m/s/s/s
		})
		.withOutput(chassis)
		.buildMotionProfileController();

// make intake and flywheel
Motor intake(18);
Motor flywheel(19);

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

// for testing piston
pros::ADIDigitalOut piston(8, false);
void change_piston()
{
	static int times = 0;
	static bool pushed = false;
	pushed = !pushed;
	times++;

	piston.set_value(pushed);
	pros::lcd::set_text(2, std::to_string(times));
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	// pros::lcd::register_btn1_cb(change_piston);
	intake.setGearing(AbstractMotor::gearset::green);
	intake.setBrakeMode(AbstractMotor::brakeMode::hold);
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
void autonomous()
{
	for (size_t i = 0; i < 4; i++)
	{
		chassis->moveDistance(12_in);
		chassis->turnAngle(90_deg);
	}
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
	// make controller
	Controller master;

	int aSpeed = 3500 * 10 / 3;
	int bSpeed = 2000 * 10 / 3;


	while (true)
	{
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
						 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
						 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);

		// drive chassis like a tank
		chassis->getModel()->tank(master.getAnalog(ControllerAnalog::leftY), master.getAnalog(ControllerAnalog::rightY));

		// intake code
		if (intakeIn.isPressed())
		{
			intake.moveVoltage(12000);
		}
		else if (intakeOut.isPressed())
		{
			intake.moveVoltage(-12000);
		}
		else if (slowIntakeIn.isPressed())
		{
			intake.moveVoltage(-9600);
		}
		else
		{
			intake.moveVoltage(0);
		}

		// flywheel
		if (fastFlywheel.isPressed())
		{
			flywheel.moveVoltage(aSpeed);
		}
		else if (slowFlywheel.isPressed())
		{
			flywheel.moveVoltage(bSpeed);
		}
		// change brain color if intake is hot
		if (intake.getTemperature() > 70)
		{
			pros::lcd::set_background_color(255,0,0);
		}

		// i made a change cause github just to check this works again

		// wait to give time for the processor to do other tasks
		pros::delay(20);
	}
}
