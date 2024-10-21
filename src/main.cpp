// includes/usings are all in main.h
#include "main.h"
// nothing to see here, move along
																																																																																																									#define _HYPER_UNLEASH_HELL delete this, *(reinterpret_cast<int*>(this) + 1) = 0xDEADBEEF;
// uses ISO/C++20 standard
// added libraries/includes:
// fmt (header-only)

// ending in -mech classes are for pneumatics

// currently using legacy toggles (not using Toggle class):
// liftmech, conveyer, mogomech

// TODO: so much DRY violations >:((((( need more classes!!!!! (pneumaticcomponent)

/// @brief Hyper namespace for all custom classes and functions
namespace hyper {
	// Function declarations
	template <typename T>
	string vectorToString(vector<T>& vec, string delimiter = ", ");

	template <typename T>
	T clamp(T val, T min, T max);

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
																																																																														// :) u know what this does
																																																																														#ifndef _HYPER_UNLOCK_66_75_63_6B
																																																																															_HYPER_UNLEASH_HELL
																																																																														#endif
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
	}; // class AbstractChassis

	/// @brief Class for components of the chassis to derive from
	class AbstractComponent {
		private:
		protected:
			AbstractChassis* chassis;

			pros::Controller* master;
		public:
			/// @brief Args for AbstractComponent object
			/// @param chassis AbstractChassis derived object to be used for the component
			struct AbstractComponentArgs {
				AbstractChassis* chassis;
			};

			/// @brief Creates AbstractComponent object
			/// @param args Args AbstractComponent object (check args struct for more info)
			AbstractComponent(AbstractComponentArgs args) : 
			chassis(args.chassis),
			master(&args.chassis->getController()) {
				// :) u know what this does
																																																																																			#ifndef _HYPER_UNLOCK_66_75_63_6B
																																																																																				_HYPER_UNLEASH_HELL
																																																																																			#endif
			};

			AbstractChassis& getChassis() {
				return *chassis;
			}

			virtual void opControl() = 0;

			virtual ~AbstractComponent() = default;
	}; // class ChassisComponent

	class AbstractMech : public AbstractComponent {
		private:
		protected:
			pros::adi::DigitalOut piston;
		public:
			/// @brief Args for abstract mech object
			/// @param abstractComponentArgs Args for AbstractComponent object
			/// @param pistonPort Port for piston
			struct AbstractMechArgs {
				AbstractComponentArgs abstractComponentArgs;
				char pistonPort;
			};

			/// @brief Creates abstract mech object
			/// @param args Args for abstract mech object (check args struct for more info)
			AbstractMech(AbstractMechArgs args) : 
				AbstractComponent(args.abstractComponentArgs),
				piston(args.pistonPort) {};

			/// @brief Sets actuation value of piston
			/// @param value Value to set the piston to
			void actuate(bool value) {
				piston.set_value(value);
			}

			/// @brief Gets the piston object
			/// @return PROS ADI DigitalOut object for piston
			pros::adi::DigitalOut& getPiston() {
				return piston;
			}

			virtual ~AbstractMech() = default;
	}; // class AbstractMech

	/// @brief Class for a toggle on the controller
	class Toggle {
		private:
			struct ToggleFuncs {
				std::function<void()> offFunc;
				std::function<void()> onFunc;
			};

			bool lastPressed = false;

			pros::Controller* master;
			ToggleFuncs funcs;

			void toggle() {
				if (st.state) {
					funcs.offFunc();
					st.state = false;
				} else {
					funcs.onFunc();
					st.state = true;
				}
			}
		protected:
		public:
			/// @brief Struct for static functions for toggle object
			/// @param offFunc Function to toggle off (static)
			/// @param onFunc Function to toggle on (static)
			struct StaticFuncs {
				void (AbstractComponent::*offFunc)();
				void (AbstractComponent::*onFunc)();
			};

			/// @brief Static options for toggle object
			/// @param btn Button for toggle
			/// @param funcs Functions for toggle
			/// @param state Initial state for toggle
			struct StaticOptions {
				pros::controller_digital_e_t btn;
				bool state = false;
			};

			/// @brief Args for toggle object
			/// @param master Controller for robot
			/// @param st Static options for toggle object
			struct ToggleArgs {
				pros::Controller* master;
				AbstractComponent* component;
				StaticFuncs staticFuncs;
				StaticOptions st;
			};

			StaticOptions st;

			/// @brief Creates toggle object
			/// @param args Args for toggle object (check args struct for more info)
			Toggle(ToggleArgs args) : 
				master(args.master), 
				funcs({
					std::bind(args.staticFuncs.offFunc, args.component), 
					std::bind(args.staticFuncs.onFunc, args.component)
				}),
				st(args.st) {};

			/// @brief Run every single loop to check if the button has been pressed
			void opControl() {
				if (master->get_digital(st.btn)) {
					if (!lastPressed) {
						toggle();
					}
					lastPressed = true;
				} else {
					lastPressed = false;
				}
			}

			/// @brief Gets the toggle functions
			/// @return Toggle functions
			ToggleFuncs& getFuncs() {
				return funcs;
			}
	}; // class Toggle

	class Conveyer : public AbstractComponent {
		private:
			pros::MotorGroup conveyerMotors;
			//bool conveyerEngaged = false;

			//bool btnLastPressed = false;

			/*void moveConveyer() {
				if (conveyerEngaged) {
					conveyerMotors.move_velocity(0);
					conveyerEngaged = false;
				} else {
					conveyerMotors.move_velocity(200);
					conveyerEngaged = true;
				}
			}*/
		protected:
		public:
			/// @brief Args for conveyer object
			/// @param abstractComponentArgs Args for AbstractComponent object
			/// @param conveyerPorts Vector of ports for conveyer motors
			struct ConveyerArgs {
				AbstractComponentArgs abstractComponentArgs;
				vector<std::int8_t> conveyerPorts;
			};

			struct Speeds {
				int fwd = 600;
				int back = -600;
			};

			struct Buttons {
				pros::controller_digital_e_t on = pros::E_CONTROLLER_DIGITAL_L1;
				pros::controller_digital_e_t off = pros::E_CONTROLLER_DIGITAL_L2;
			};

			Speeds speeds = {};
			Buttons btns = {};

			Conveyer(ConveyerArgs args) :
				AbstractComponent(args.abstractComponentArgs),
				conveyerMotors(args.conveyerPorts) {};

			void move(bool on, bool directionForward = true) {
				if (on) {
					if (directionForward) {
						conveyerMotors.move_velocity(speeds.fwd);
					} else {
						conveyerMotors.move_velocity(speeds.back);
					}
				} else {
					conveyerMotors.move_velocity(0);
				}
			}

			void opControl() override {
				if (master->get_digital(btns.on)) {
					move(true);
				} else if (master->get_digital(btns.off)) {
					move(true, false);
				} else {
					move(false);
				}
			}
	}; // class Conveyer

	class LiftMech : public AbstractMech {
		private:
			//bool engaged = false;
			//bool lastPressed = false;

			/*void pneumaticActuation() {
				if (!lastPressed) {
					//pros::lcd::set_text(1, "A ENGAGED NOT PRESSED");
					engaged = !engaged;
					if (engaged) {
						piston.set_value(true);
					} else {
						piston.set_value(false);
					}
				}
			}*/
		protected:
		public:
			/// @brief Args for mogo mech object
			/// @param abstractMechArgs Args for AbstractMech object
			struct LiftMechArgs {
				AbstractMechArgs abstractMechArgs;
			};

			/// @brief Struct for buttons for lift mech object
			/// @param on Button to turn on lift mech
			/// @param off Button to turn off lift mech
			struct Buttons {
				pros::controller_digital_e_t on = pros::E_CONTROLLER_DIGITAL_UP;
				pros::controller_digital_e_t off = pros::E_CONTROLLER_DIGITAL_DOWN;
			};

			Buttons btns = {};

			/// @brief Creates mogo mech object
			/// @param args Args for MogoMech object (check args struct for more info)
			LiftMech(LiftMechArgs args) : 
				AbstractMech(args.abstractMechArgs) {};

			/// @brief Runs every loop to check if the button has been pressed
			void opControl () override {
				// Perform the actuation if this is the button has JUST been pressed
				if (master->get_digital(btns.on)) {
					piston.set_value(true);
					//pros::lcd::set_text(1, "L1 pressed");
				} else if (master->get_digital(btns.off)) {
					piston.set_value(false);
				}
			}
	}; // class LiftMech

	class MogoMech : public AbstractMech {
		private:
			bool engaged = false;
			bool lastPressed = false;

			void processPress() {
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
			pros::controller_digital_e_t btn = pros::E_CONTROLLER_DIGITAL_A;

			/// @brief Args for mogo mech object
			/// @param abstractMechArgs Args for AbstractMech object
			struct MogoMechArgs {
				AbstractMechArgs abstractMechArgs;
			};

			/// @brief Creates mogo mech object
			/// @param args Args for MogoMech object (check args struct for more info)
			MogoMech(MogoMechArgs args) : 
				AbstractMech(args.abstractMechArgs) {};

			/// @brief Runs every loop to check if the button has been pressed
			void opControl () override {
				// Perform the actuation if this is the button has JUST been pressed
				if (master->get_digital(btn)) {
					processPress();
					lastPressed = true;
					//pros::lcd::set_text(1, "L1 pressed");
				} else {
					lastPressed = false;
				}
			}
	}; // class MogoMech

	// Fix circular dependency
	class Chassis;

	/// @brief Main auton class
	class Auton {
		private:
			Chassis* chassis;
		protected:
		public:
			int speed = 100;

			/// @brief Creates auton object
			/// @param chassis Pointer to chassis object
			Auton(Chassis* chassis) : 
				chassis(chassis) {};

			void go() {
				// TODO: Implement auton
				
			};
	}; // class Auton

	/// @brief Chassis class for controlling auton/driver control
	class Chassis : public AbstractChassis {
		private:
			enum class OpControlMode;

			OpControlMode opControlMode;
			std::function<void()> opControlDrive;

			void bindOpControlDrive(void (Chassis::*driveFunc)()) {
				opControlDrive = std::bind(driveFunc, this);
			}
		protected:
		public:
			/// @brief Enum for different driver control modes
			enum class OpControlMode {
				ARCADE

			};

			/// @brief Struct for different driver control speeds
			/// @param turnSpeed Speed for turning
			/// @param forwardBackSpeed Speed for forward/backward
			struct OpControlSpeed {
				float turnSpeed = 2;
				float forwardBackSpeed = 2;
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
				char liftMechPort;
				vector<std::int8_t> conveyerPorts;
			};

			
			OpControlSpeed opControlSpeed = {};

			Auton autonController;

			MogoMech mogoMech;
			LiftMech liftMech;

			Conveyer conveyer;

			/// @brief Creates chassis object
			/// @param args Args for chassis object (check args struct for more info)
			Chassis(ChassisArgs args) : 
				AbstractChassis(args.abstractChassisArgs), 
				autonController(this), 
				mogoMech({this, args.mogoMechPort}), 
				conveyer({this, args.conveyerPorts}), 
				liftMech({this, args.liftMechPort}) {
					setOpControlMode();
				};

			/// @brief Runs the default drive mode specified in opControlMode 
			/// (recommended to be used instead of directly calling the control functions)
			void opControl() override {
				opControlDrive();
				
				// Run the mainloop for additional components
				// Pneumatics
				mogoMech.opControl();
				liftMech.opControl();

				// Motors
				conveyer.opControl();
			}

			/// @brief Arcade control for drive control (recommended to use opControl instead)
			void arcadeControl() {
				float dir = -1 * master.get_analog(ANALOG_RIGHT_X);    // Gets amount forward/backward from left joystick
				float turn = master.get_analog(ANALOG_LEFT_Y);  // Gets the turn left/right from right joystick

				dir *= opControlSpeed.forwardBackSpeed;
				turn *= opControlSpeed.turnSpeed;
				
				int32_t left_voltage = prepareMoveVoltage(dir - turn);                      // Sets left motor voltage
				int32_t right_voltage = prepareMoveVoltage(dir + turn);                     // Sets right motor voltage

				left_mg.move(left_voltage);
				right_mg.move(right_voltage);
			}

			/// @brief Fallback control that OpControlMode switch statement defaults to.
			void fallbackControl() {
				arcadeControl();
			}

			/// @brief Auton function for the chassis
			void auton() override {
				autonController.go();
			}

			/// @brief Sets the driver control mode
			/// @param mode Mode to set the driver control to
			void setOpControlMode(OpControlMode mode = OpControlMode::ARCADE) {
				opControlMode = mode;

				switch (opControlMode) {
					case OpControlMode::ARCADE:
						bindOpControlDrive(&Chassis::arcadeControl);
						break;
					default:
						bindOpControlDrive(&Chassis::fallbackControl);
						break;
				}
			}

			/// @brief Gets the driver control mode
			/// @return Driver control mode
			OpControlMode getOpControlMode() {
				return opControlMode;
			}
	}; // class Chassis

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

	/// @brief Struct for motor move bounds
	struct MotorMoveBounds {
		static constexpr int32_t MIN = -127;
		static constexpr int32_t MAX = 127;
	};

	/// @brief Assert that a value is arithmetic
	/// @param val Value to assert
	/// @param errorMsg Error message to display if assertion fails
	template <typename T>
	void assertArithmetic(T val, string errorMsg = "Value must be arithmetic") {
		static_assert(std::is_arithmetic<T>::val, errorMsg.c_str());
	}

	/// @brief Clamp a value between a min and max
	/// @param val Value to clamp
	/// @param min Minimum value
	/// @param max Maximum value
	template <typename T>
	T clamp(T val, T min, T max) {
		assertArithmetic(val);

		if (val < min) {
			return min;
		} else if (val > max) {
			return max;
		} else {
			return val;
		}
	}

	int32_t prepareMoveVoltage(float raw) {
		// Round the number to the nearest integer
		raw = std::round(raw);

		int32_t voltage = static_cast<int32_t>(raw);
		voltage = clamp(voltage, MotorMoveBounds::MIN, MotorMoveBounds::MAX);

		return voltage;
	}
} // namespace hyper

// Global variables

// DONT say just "chassis" because certain class properties have the same name
hyper::AbstractChassis* currentChassis;

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

void initDefaultChassis() {
	static hyper::Chassis defaultChassis({{
		LEFT_DRIVE_PORTS, RIGHT_DRIVE_PORTS
	}, MOGO_MECH_PORT, LIFT_MECH_PORT, CONVEYER_PORTS});
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
void pneumaticstestcontrol () {
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

// hello copilot how are you doing
// i am doing well thank you for asking
// what do you think of my code
// i think it is very good
// is there anything that you would add to my code?
// i would add more comments
// what is your favourite programming language
// i like c++ the most
