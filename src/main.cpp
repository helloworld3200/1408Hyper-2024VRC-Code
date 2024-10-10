// includes/usings are all in main.h
#include "main.h"

// uses ISO/C++20 standard
// added libraries/includes:
// fmt (header-only)

// Function declarations
template <typename T>
string vectorToString(vector<T>& vec, string delimiter = ", ");

/// @brief Abstract chassis class for if you want a custom chassis class
class AbstractChassis {
	private:
	protected:
		pros::MotorGroup left_mg;
		pros::MotorGroup right_mg;
		pros::Controller master;
	public:
		/// @brief Args for abstract chassis object
		/// @param leftPorts Vector of ports for left side of drivetrain
		/// @param rightPorts Vector of ports for right side of drivetrain
		/// @param master Controller for robot
		struct AbstractChassisArgs {
			std::vector<std::int8_t> leftPorts;
			std::vector<std::int8_t> rightPorts;
			pros::controller_id_e_t master = pros::E_CONTROLLER_MASTER;
		};

		/// @brief Creates abstract chassis object
		/// @param args Args for abstract chassis object (check args struct for more info)
		AbstractChassis(AbstractChassisArgs args) : 
		  left_mg(args.leftPorts), right_mg(args.rightPorts), 
		  master(args.master) {
			string consoleMsg = fmt::format("Chassis created with left ports: {} and right ports: {}",
			 vectorToString(args.leftPorts), vectorToString(args.rightPorts));
			pros::lcd::print(0, consoleMsg.c_str());
		};

		virtual ~AbstractChassis() = default;

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

/// @brief Class for components of the chassis to derive from
class ChassisComponent {
	private:
	protected:
		AbstractChassis* chassis;

		pros::Controller* master;
	public:
		/// @brief Args for ChassisComponent object
		/// @param chassis AbstractChassis derived object to be used for the component
		struct ChassisComponentArgs {
			AbstractChassis* chassis;
		};

		/// @brief Creates ChassisComponent object
		/// @param args Args ChassisComponent object (check args struct for more info)
		ChassisComponent(ChassisComponentArgs args) : 
		chassis(args.chassis),
		master(&args.chassis->getController()) {};

		AbstractChassis& getChassis() {
			return *chassis;
		}

		virtual ~ChassisComponent() = default;
};

class Conveyer : public ChassisComponent {
	private:
		pros::MotorGroup conveyerMotors;
		bool conveyerEngaged = false;

		bool btnLastPressed = false;
		pros::controller_digital_e_t btn = pros::E_CONTROLLER_DIGITAL_R1;

		void moveConveyer() {
			if (conveyerEngaged) {
				conveyerMotors.move_velocity(0);
				conveyerEngaged = false;
			} else {
				conveyerMotors.move_velocity(200);
				conveyerEngaged = true;
			}
		}
	protected:
	public:
		struct ConveyerArgs {
			ChassisComponentArgs chassisComponentArgs;
			vector<std::int8_t> conveyerPorts;
		};

		Conveyer(ConveyerArgs args) :
			ChassisComponent(args.chassisComponentArgs),
			conveyerMotors(args.conveyerPorts) {};

		void opControl() {
			if (master->get_digital(btn)) {
				if (!btnLastPressed) {
					moveConveyer();
				}

				btnLastPressed = true;
			} else {
				btnLastPressed = false;
			}
		}
};

class ConvMech : public ChassisComponent {
	private:
		pros::ADIDigitalOut piston;
		bool engaged = false;
		bool lastPressed = false;

		void pneumaticActuation() {
			if (!lastPressed) {
				//pros::lcd::set_text(1, "A ENGAGED NOT PRESSED");
				engaged = !engaged;
				if (engaged) {
					piston.set_value(true);
				} else {
					piston.set_value(false);
				}
			}
		}
	protected:
	public:
		pros::controller_digital_e_t btn;

		/// @brief Args for mogo mech object
		/// @param chassisComponentArgs Args for ChassisComponent object
		struct ConvMechArgs {
			ChassisComponentArgs chassisComponentArgs;
			char pistonPort;
			pros::controller_digital_e_t btn = pros::E_CONTROLLER_DIGITAL_B;
		};

		/// @brief Creates mogo mech object
		/// @param args Args for MogoMech object (check args struct for more info)
		ConvMech(ConvMechArgs args) : 
			ChassisComponent(args.chassisComponentArgs),
			piston(args.pistonPort),
			btn(args.btn) {};

		void opControl () {
			// Perform the actuation if this is the button has JUST been pressed
			if (master->get_digital(btn)) {
				pneumaticActuation();
				lastPressed = true;
				//pros::lcd::set_text(1, "L1 pressed");
			} else {
				lastPressed = false;
			}
		}

		pros::ADIDigitalOut& getPiston() {
			return piston;
		}
};

class MogoMech : public ChassisComponent {
	private:
		pros::ADIDigitalOut piston;
		bool engaged = false;
		bool lastPressed = false;

		void pneumaticActuation() {
			if (!lastPressed) {
				//pros::lcd::set_text(1, "A ENGAGED NOT PRESSED");
				engaged = !engaged;
				if (engaged) {
					piston.set_value(true);
				} else {
					piston.set_value(false);
				}
			}
		}
	protected:
	public:
		/// @brief Args for mogo mech object
		/// @param chassisComponentArgs Args for ChassisComponent object
		struct MogoMechArgs {
			ChassisComponentArgs chassisComponentArgs;
			char pistonPort;
		};

		/// @brief Creates mogo mech object
		/// @param args Args for MogoMech object (check args struct for more info)
		MogoMech(MogoMechArgs args) : 
			ChassisComponent(args.chassisComponentArgs),
			piston(args.pistonPort) {};

		void opControl () {
			// Perform the actuation if this is the button has JUST been pressed
			if (master->get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
				pneumaticActuation();
				lastPressed = true;
				//pros::lcd::set_text(1, "L1 pressed");
			} else {
				lastPressed = false;
			}
		}

		pros::ADIDigitalOut& getPiston() {
			return piston;
		}
};

/// @brief Main auton class
class Auton : public ChassisComponent {
	private:
	public:
		/// @brief Args for auton object
		/// @param chassisComponentArgs Args for ChassisComponent object
		/// @param speed Speed for auton control
		struct AutonArgs {
			ChassisComponentArgs chassisComponentArgs;
			int speed = 100;
		};

		int speed;

		/// @brief Creates auton object
		/// @param args Args for ChassisComponent object (check args struct for more info)
		Auton(AutonArgs args) : 
			ChassisComponent(args.chassisComponentArgs), speed(args.speed) {};

		void go() {
			// TODO: Implement auton
			
		};
};

/// @brief Chassis class for controlling auton/driver control
class Chassis : public AbstractChassis {
	private:
	public:
		/// @brief Enum for different driver control modes
		enum class OpControlMode {
			ARCADE

		};

		/// @brief Struct for different driver control speeds
		/// @param turnSpeed Speed for turning
		/// @param forwardBackSpeed Speed for forward/backward
		struct OpControlSpeed {
			int turnSpeed = 2;
			int forwardBackSpeed = 2;
		};

		/// @brief Args for chassis object
		/// @param abstractChassisArgs Args for abstract chassis object
		/// @param opControlMode Mode for driver control
		/// @param opControlSpeed Speed for driver control
		/// @param autonSpeed Speed for auton control
		/// @param mogoMechPort Port for mogo mech
		struct ChassisArgs {
			AbstractChassisArgs abstractChassisArgs;
			char mogoMechPort;
			vector<std::int8_t> conveyerPorts;
			OpControlSpeed opControlSpeed = {};
			OpControlMode opControlMode = OpControlMode::ARCADE;
			int autonSpeed = 100;
		};

		OpControlMode opControlMode;
		OpControlSpeed opControlSpeed;

		Auton autonController;

		MogoMech mogoMech;
		
		Conveyer conveyer;

		/// @brief Creates chassis object
		/// @param args Args for chassis object (check args struct for more info)
		Chassis(ChassisArgs args) : 
		  AbstractChassis(args.abstractChassisArgs), 
		  opControlMode(args.opControlMode), 
		  opControlSpeed(args.opControlSpeed), 
		  autonController({this}),
		  mogoMech({this, args.mogoMechPort}),
		  conveyer({this, args.conveyerPorts}) {};

		/// @brief Runs the default drive mode specified in opControlMode 
		/// (recommended to be used instead of directly calling the control functions)
		void opControl() override {
			// Run the mainloop for the mogo mech
			mogoMech.opControl();

			switch (opControlMode) {
				case OpControlMode::ARCADE:
					arcadeControl();
					break;
				default:
					arcadeControl();
					break;
			}
		}

		/// @brief Arcade control for drive control (recommended to use opControl instead)
		void arcadeControl() {
			int dir = -1 * master.get_analog(ANALOG_RIGHT_X);    // Gets amount forward/backward from left joystick
			int turn = master.get_analog(ANALOG_LEFT_Y);  // Gets the turn left/right from right joystick

			dir *= opControlSpeed.forwardBackSpeed;
			turn *= opControlSpeed.turnSpeed;
			
			int left_voltage = dir - turn;                      // Sets left motor voltage
			int right_voltage = dir + turn;                     // Sets right motor voltage

			left_mg.move(left_voltage);
			right_mg.move(right_voltage);
		}

		/// @brief Auton function for the chassis
		void auton() override {
			autonController.go();
		}
};

// DONT say just "chassis" because certain class properties have the same name
AbstractChassis* currentChassis;

/// @brief Convert vector of ints to string. For displaying on the LCD/debugging
/// @param vec Vector to convert
/// @param delimiter Delimiter to separate elements
template <typename T>
string vectorToString(vector<T>& vec, string delimiter) {
	int vecSize = vec.size();
	int vecSizeMinusOne = vecSize - 1;
	std::ostringstream oss;

	oss << "{";
	for (int i = 0; i < vecSize; i++) {
		oss << vec[i];
		if (i < vecSizeMinusOne) {
			oss << delimiter;
		}
	}
	oss << "}";

	return oss.str();
}

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

void initDefaultChassis() {
	static Chassis defaultChassis({{
		LEFT_DRIVE_PORTS, RIGHT_DRIVE_PORTS
	}, MOGO_MECH_PORT, CONVEYER_PORTS});
	currentChassis = &defaultChassis;
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

	INIT_CHASSIS();

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
 * starts
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
	if (DO_AUTON) {
		currentChassis->auton();
	}
}



// Not used anymore, used to be for pneumatics testing
void testcontrol () {
	pros::lcd::set_text(0, "In testcontrol");
	pros::Controller controller(pros::E_CONTROLLER_MASTER);
	while (true) {

		pros::delay(20); // Add a small delay to prevent overwhelming the CPU
	}
}

void mainControl() {
	if (AUTON_TEST) {
		autonomous();
	}

	pros::Controller controller(pros::E_CONTROLLER_MASTER);

	bool opControlRunning = DO_OP_CONTROL;
	// Chassis control loop
	while (opControlRunning) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
						 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
						 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0); // Prints status of the emulated screen LCDs

		// Chassis opcontrol
		currentChassis->opControl();



		pros::delay(DELAY_TIME_MS); // Run for 20 ms then update
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

void opcontrol() {
	CURRENT_OPCONTROL();
}
