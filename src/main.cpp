#include "main.h"

// make controller buttons
ControllerButton intakeIn(ControllerDigital::R2);
ControllerButton intakeOut(ControllerDigital::R1);

ControllerButton fastFlywheel(ControllerDigital::A);
ControllerButton slowFlywheel(ControllerDigital::B);
ControllerButton flywheelStop(ControllerDigital::up);

ControllerButton angleChange(ControllerDigital::Y);
ControllerButton launchEndgame(ControllerDigital::X);

// make chassis
std::shared_ptr<OdomChassisController> chassis =
	ChassisControllerBuilder()
		.withMotors(
			{-12, -14, 8},
			{13, 15, -18})
		// Green gearset, 4 in wheel diam, 11.5 in wheel track
		.withDimensions({AbstractMotor::gearset::green, (36.0 / 60.0)}, {{3.25_in, 11.5_in}, imev5GreenTPR})
		/*/ PID
		.withGains(
			{0.45, 0.1, 0.1}, // distance controller gains
			{0.45, 0.1, 0.1}  // turn controller gains // angle controller gains (helps drive straight)
			)				  //*/
		.withOdometry()
		.buildOdometry();

// make path generator | copied from okapi tutorials

// make intake and flywheel
pros::Motor intake(7);
pros::Motor flywheel(19);

// make angle changer
bool angled = false;
pros::ADIDigitalOut AngleChanger('h', angled);

pros::ADIDigitalOut endgame('a', false);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
double integral = 0;
double prevError = 0;
double maxSpeed = 0;

const float kP = 1;
const float kI = 0.1;
const float kD = 0.2;

int pid(double speed, double target)
{
	const float dt = 0.01;
	double error = target - speed;
	integral += error;
	if (integral > target || error == 0)
	{
		integral = 0;
	}
	else if (error < -100 || error > 100)
	{
		integral = 0;
	}
	// double derivative = error - prevError;
	// prevError = error;

	return speed + kP * error + kI * integral; // + kD * derivative;
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
	flywheel.set_gearing(MOTOR_GEARSET_06);
	flywheel.set_brake_mode(MOTOR_BRAKE_COAST);
	intake.set_gearing(MOTOR_GEARSET_18);
	intake.set_brake_mode(MOTOR_BRAKE_HOLD);
	// intake.set_pos_pid((0.1, 0.1, 0, 0));
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

bool auto_done = false;
void set_flywheel()
{
	while (!auto_done)
	{
		flywheel.move_velocity(pid(flywheel.get_actual_velocity(), 615));
		pros::delay(10);
	}
}

void autonomous()
{
	pros::Task my_task(set_flywheel);

	chassis->setMaxVelocity(25);

	chassis->setState({0_in, 0_in, 0_deg});

	chassis->driveToPoint({-1.5_in, 0_in}, true);
	chassis->waitUntilSettled();

	intake.move_relative(-600, 200);
	pros::delay(500);

	chassis->setMaxVelocity(50);

	chassis->moveDistanceAsync(8_in);
	intake.move_velocity(-200);
	chassis->waitUntilSettled();

	chassis->turnAngle(-22.25_deg);

	pros::delay(500);

	intake.move_relative(400, 200);

	pros::delay(1000);

	intake.move_relative(10000, 200);

	pros::delay(100);
	auto_done = true;
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
			intake.move_voltage(-12000);
		}
		else if (intakeOut.isPressed())
		{
			intake.move_voltage(5750);
		}
		else
		{
			intake.move_voltage(0);
		}

		// flywheel
		if (fastFlywheel.isPressed())
		{ // max speed
			target = 650.0;
		}
		else if (slowFlywheel.isPressed())
		{ // 2.5k rpm
			target = 560.0;
			// pros::lcd::set_background_color(255, 0, 0);
		}
		else if (flywheelStop.isPressed())
		{
			target = 0; // flywheel is just going to keep on spinning
			flywheel.move_voltage(0);
		}
		// change brain color if intake is hot
		if (intake.get_temperature() > 70)
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

		if (launchEndgame.changedToPressed())
		{
			endgame.set_value(true);
		}

		if (target != 0)
		{
			flywheel.move_velocity(pid(flywheel.get_actual_velocity(), target));
		}

		if (flywheel.get_actual_velocity() > maxSpeed)
		{
			maxSpeed = flywheel.get_actual_velocity();
		}

		// print flywheel speed
		pros::lcd::set_text(6, std::to_string(flywheel.get_actual_velocity()));
		pros::lcd::set_text(7, std::to_string(maxSpeed));
		pros::lcd::set_text(5, std::to_string(target));

		// print intake temperature
		pros::lcd::set_text(4, std::to_string(intake.get_temperature()));

		// wait to give time for the processor to do other tasks
		pros::delay(20);
	}
}
