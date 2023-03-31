#include "main.h"

// make controller buttons
ControllerButton intakeIn(ControllerDigital::R2);
ControllerButton intakeOut(ControllerDigital::R1);

ControllerButton fastFlywheel(ControllerDigital::A);
ControllerButton slowFlywheel(ControllerDigital::B);
ControllerButton flywheelStop(ControllerDigital::up);

ControllerButton angleChange(ControllerDigital::Y);

// make chassis
std::shared_ptr<OdomChassisController> chassis =
	ChassisControllerBuilder()
		.withMotors(
			{-12, -14, 8},
			{13, 15, -18})
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
Motor intake(7);
Motor flywheel(19);

// make angle changer
bool angled = false;
pros::ADIDigitalOut AngleChanger('h', angled);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

int pid(double speed, double target)
{
	const float kP = 0.1;
	double error = target - speed;
	return speed + kP * error;
}

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
void initialize()
{
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	// pros::lcd::register_btn1_cb(change_piston);
	intake.setGearing(AbstractMotor::gearset::blue);
	intake.setBrakeMode(AbstractMotor::brakeMode::hold);

	flywheel.setBrakeMode(AbstractMotor::brakeMode::coast);
	flywheel.setGearing(AbstractMotor::gearset::blue);
	flywheel.setVelPID(0.0075, 1, 0, 0);
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

	int aSpeed = 3600 * 10 / 3;
	int bSpeed = 3000 * 10 / 3;

	double target = 0.0;

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
		else
		{
			intake.moveVoltage(0);
		}

		// flywheel
		if (fastFlywheel.isPressed())
		{ // max speed
			target = 600.0;
		}
		else if (slowFlywheel.isPressed())
		{ // 2.5k rpm
			target = 2500 / 6;
		}
		else if (flywheelStop.isPressed())
		{
			target = 0; // flywheel is just going to keep on spinning
			flywheel.moveVoltage(0);
		}
		// change brain color if intake is hot
		if (intake.getTemperature() > 70)
		{
			pros::lcd::set_background_color(255, 0, 0);
		}

		// angle changer
		if (angleChange.changedToPressed())
		{
			angled = !angled;
			pros::lcd::set_text(7, "sdfsd");
			AngleChanger.set_value(angled);
		}

		if (target != 0)
		{
			flywheel.moveVelocity(pid(flywheel.getActualVelocity(), target));
		}

		// print flywheel speed
		pros::lcd::set_text(6, std::to_string(flywheel.getActualVelocity()));
		pros::lcd::set_text(5, std::to_string(target));

		// print intake temperature
		pros::lcd::set_text(4, std::to_string(intake.getTemperature()));

		// wait to give time for the processor to do other tasks
		pros::delay(20);
	}
}
