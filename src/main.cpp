// includes/usings are all in main.h
#include "main.h"

// uses ISO/C++20 standard
// added libraries/includes:
// fmt (header-only)

// Variables (u can change these!!)
#define DELAY_TIME_MS 20
// turn on for auton to be run at the start of opcontrol
#define AUTON_TEST false

#define LEFT_DRIVE_PORTS {-1, 2, -3}
#define RIGHT_DRIVE_PORTS {-4, 5, -6}

#define INIT_DRIVETRAIN initDefaultDrivetrain

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
		/// @brief Args for abstract drivetrain object
		/// @param leftPorts Vector of ports for left side of drivetrain
		/// @param rightPorts Vector of ports for right side of drivetrain
		/// @param master Controller for drivetrain
		struct AbstractDrivetrainArgs {
			std::vector<std::int8_t> leftPorts;
			std::vector<std::int8_t> rightPorts;
			pros::controller_id_e_t master = pros::E_CONTROLLER_MASTER;
		};

		/// @brief Creates abstract drivetrain object
		/// @param args Args for abstract drivetrain object (check args struct for more info)
		AbstractDrivetrain(AbstractDrivetrainArgs args) : 
		  left_mg(args.leftPorts), right_mg(args.rightPorts), master(args.master) {
			string consoleMsg = fmt::format("Drivetrain created with left ports: {} and right ports: {}",
			 vectorToString(args.leftPorts), vectorToString(args.rightPorts));
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
		/// @brief Args for abstract auton object
		/// @param drivetrain Drivetrain object for auton control
		struct AbstractAutonArgs {
			AbstractDrivetrain* drivetrain;
		};

		/// @brief Creates abstract auton object
		/// @param args Args for abstract auton object (check args struct for more info)
		AbstractAuton(AbstractAutonArgs args) : drivetrain(args.drivetrain) {};
		virtual ~AbstractAuton() = default;
		virtual void go() = 0;
};

/// @brief Main auton class
class Auton : public AbstractAuton {
	private:
	public:
		/// @brief Args for auton object
		/// @param abstractAutonArgs Args for abstract auton object
		/// @param speed Speed for auton control
		struct AutonArgs {
			AbstractAuton::AbstractAutonArgs abstractAutonArgs;
			int speed = 100;
		};

		int speed;

		/// @brief Creates auton object
		/// @param args Args for auton object (check args struct for more info)
		Auton(AutonArgs args) : 
			AbstractAuton(args.abstractAutonArgs), speed(args.speed) {};

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

		/// @brief Args for drivetrain object
		/// @param abstractDrivetrainArgs Args for abstract drivetrain object
		/// @param opControlMode Mode for driver control
		/// @param opControlSpeed Speed for driver control
		/// @param autonSpeed Speed for auton control
		struct DrivetrainArgs {
			AbstractDrivetrain::AbstractDrivetrainArgs abstractDrivetrainArgs;
			OpControlMode opControlMode = OpControlMode::ARCADE;
			int opControlSpeed = 1;
			int autonSpeed = 100;
		};

		Drivetrain::OpControlMode opControlMode;
		int opControlSpeed;

		Auton autonController;

		/// @brief Creates drivetrain object
		/// @param args Args for drivetrain object (check args struct for more info)
		Drivetrain(DrivetrainArgs args) : 
		  AbstractDrivetrain(args.abstractDrivetrainArgs), 
		  opControlMode(args.opControlMode), 
		  opControlSpeed(args.opControlSpeed), 
		  autonController({this}) {};

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

// DONT say just "drivetrain" because certain class properties have the same name
AbstractDrivetrain* currentDrivetrain;

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

void initDefaultDrivetrain() {
	static Drivetrain defaultDrivetrain({{
		LEFT_DRIVE_PORTS, RIGHT_DRIVE_PORTS
	}});
	currentDrivetrain = &defaultDrivetrain;
}

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

	INIT_DRIVETRAIN();

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
	currentDrivetrain->auton();
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

	if (AUTON_TEST) {
		autonomous();
	}
	
	// Drivetrain control loop
	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0); // Prints status of the emulated screen LCDs


		// Drivetrain control
		currentDrivetrain->opControl();

		pros::delay(DELAY_TIME_MS); // Run for 20 ms then update
	}
}