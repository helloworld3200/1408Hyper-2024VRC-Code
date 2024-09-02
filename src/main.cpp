// includes/usings are all in main.h
#include "main.h"

// uses ISO/C++20 standard
// added libraries/includes:
// fmt (header-only)

/// @brief Convert vector of ints to string. For displaying on the LCD/debugging
/// @param vec Vector to convert
/// @param delimiter Delimiter to separate elements
template <typename T>
string vectorToString(vector<T>& vec, string delimiter = ", ") {
	std::ostringstream oss;

	oss << "{";
	for (int i = 0; i < vec.size(); i++) {
		oss << vec[i];
		if (i < vec.size() - 1) {
			oss << delimiter;
		}
	}
	oss << "}";

	return oss.str();
}

/// @brief Abstract drivetrain class for if you want a custom drivetrain class
class AbstractDrivetrain {
	private:
	protected:
		pros::MotorGroup left_mg;
		pros::MotorGroup right_mg;
		pros::Controller master;
	public:
		/// @brief Creates abstract drivetrain object
		/// @param leftPorts Ports for the left motor group
		/// @param rightPorts Ports for the right motor group
		/// @param master Controller for driver control
		AbstractDrivetrain(
		  std::vector<std::int8_t>& leftPorts, 
		  std::vector<std::int8_t>& rightPorts, 
		  pros::controller_id_e_t master = pros::E_CONTROLLER_MASTER 
		) : left_mg(leftPorts), right_mg(rightPorts), master(master) {
			string consoleMsg = fmt::format("Drivetrain created with left ports: {} and right ports: {}",
			 vectorToString(leftPorts), vectorToString(rightPorts));
			pros::lcd::print(0, consoleMsg.c_str());
		};

		virtual ~AbstractDrivetrain() = default;

		// Getters for motor groups and controller

		/// @brief Gets the left motor group
		pros::MotorGroup& getLeftMotorGroup() {
			return left_mg;
		}

		/// @brief Gets the right motor group
		pros::MotorGroup& getRightMotorGroup() {
			return right_mg;
		}

		/// @brief Gets the controller
		pros::Controller& getController() {
			return master;
		}

		virtual void opControl() = 0;
		virtual void auton() = 0;
};

/// @brief Abstract class for autonomous routines
class AbstractAuton {
	private:
	protected:
		AbstractDrivetrain* drivetrain;
	public:
		/// @brief Creates abstract auton object
		/// @param drivetrain Drivetrain object to control
		AbstractAuton(AbstractDrivetrain* drivetrain) : drivetrain(drivetrain) {};
		virtual ~AbstractAuton() = default;
		virtual void go() = 0;
};

/// @brief Main auton class
class Auton : AbstractAuton {
	private:
	public:
		int speed;

		/// @brief Creates auton object
		/// @param drivetrain Drivetrain object to control
		/// @param speed Speed for the auton
		Auton(AbstractDrivetrain* drivetrain, int speed) : 
			AbstractAuton(drivetrain), speed(speed) {};

		void go() override {
			// TODO: Implement auton
		};
};

/// @brief Drivetrain class for controlling auton/driver control
class Drivetrain : public AbstractDrivetrain {
	private:
	public:
		/// @brief Enum for different driver control modes
		enum OpControlMode {
			ARCADE = 0
		};

		Drivetrain::OpControlMode opControlMode;
		int opControlSpeed;

		Auton autonController;

		/// @brief Creates drivetrain object
		/// @param leftPorts Ports for the left motor group
		/// @param rightPorts Ports for the right motor group
		/// @param master Controller for driver control
		/// @param opControlMode Default mode for driver control
		/// @param opControlSpeed Opcontrol speed multiplier (opcontrol only)
		/// @param autonSpeed Max speed for the auton (auton only)
		Drivetrain(
		  std::vector<std::int8_t>& leftPorts, 
		  std::vector<std::int8_t>& rightPorts, 
		  pros::controller_id_e_t master = pros::E_CONTROLLER_MASTER, 
		  Drivetrain::OpControlMode opControlMode = Drivetrain::OpControlMode::ARCADE,
		  int opControlSpeed = 1, 
		  int autonSpeed = 100
		  ) : AbstractDrivetrain(leftPorts, rightPorts, master), opControlMode(opControlMode), 
		  opControlSpeed(opControlSpeed), autonController(this, autonSpeed) {};

		/// @brief Runs the default drive mode specified in opControlMode 
		/// (recommended to be used instead of directly calling the control functions)
		void opControl() override {
			switch (opControlMode) {
				case Drivetrain::OpControlMode::ARCADE:
					arcadeControl();
					break;
				default:
					arcadeControl();
					break;
			}
		}

		/// @brief Arcade control for drive control (recommended to use opControl instead)
		void arcadeControl() {
			int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
			int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
			
			int left_voltage = opControlSpeed * (dir - turn);                      // Sets left motor voltage
			int right_voltage = opControlSpeed * (dir + turn);                     // Sets right motor voltage

			left_mg.move(left_voltage);
			right_mg.move(right_voltage);
		}

		/// @brief Auton function for the drivetrain
		void auton() override {
			autonController.go();
		}
};

// Constants
vector<std::int8_t> leftDrivePorts = {-1, 2, -3};
vector<std::int8_t> rightDrivePorts = {-4, 5, -6};

const int delayTimeMs = 20;

// Turn this on if we are testing auton
const bool autonTest = false;

// Main drivetrain object
Drivetrain defaultDrivetrain(leftDrivePorts, rightDrivePorts);

// If we are testing a different drivetrain change this
AbstractDrivetrain& currentDrivetrain = defaultDrivetrain;

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
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
void autonomous() {
	currentDrivetrain.auton();
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

void opcontrol() {

	if (autonTest) {
		autonomous();
	}
	
	// Drivetrain control loop
	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0); // Prints status of the emulated screen LCDs


		// Drivetrain control
		currentDrivetrain.opControl();

		pros::delay(delayTimeMs); // Run for 20 ms then update
	}
}