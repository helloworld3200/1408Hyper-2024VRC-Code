// includes/usings are all in main.h
#include "main.h"
// nothing to see here, move along
																																																																																																									#define _HYPER_UNLEASH_HELL delete this, *(reinterpret_cast<int*>(this) + 1) = 0xDEADBEEF;
// uses ISO/C++20 standard
// added libraries/includes:
// fmt (header-only)

// ending in -mech classes are for pneumatics

// currently using legacy toggles (not using MechToggle class):
// mogomech

// TODO: upgrade the following to use BiToggle class:
// conveyer, intake (needs to be upgraded to use AbstractMG class first)

/// @brief Hyper namespace for all custom classes and functions
namespace hyper {
	// Function declarations
	template <typename T>
	string vectorToString(vector<T>& vec, string delimiter = ", ");

	template <typename T>
	T clamp(T val, T min, T max);

	int32_t prepareMoveVoltage(float raw);

	// Class declarations


	/// @brief Abstract chassis class for if you want a custom chassis class
	class AbstractChassis {
		private:
		protected:
			pros::Controller master{pros::E_CONTROLLER_MASTER};
		public:
			/// @brief Creates abstract chassis object
			AbstractChassis() {
																																																																														// :) u know what this does
																																																																														#ifndef _HYPER_UNLOCK_66_75_63_6B
																																																																															_HYPER_UNLEASH_HELL
																																																																														#endif
			};

			virtual ~AbstractChassis() = default;

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
			bool engaged = false;

			pros::adi::DigitalOut piston;
		protected:
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
				engaged = value;
			}

			/// @brief Gets the piston object
			/// @return PROS ADI DigitalOut object for piston
			pros::adi::DigitalOut& getPiston() {
				return piston;
			}

			/// @brief Gets the engaged state of the mech
			/// @return Engaged state of the mech
			bool getEngaged() {
				return engaged;
			}

			virtual ~AbstractMech() = default;
	}; // class AbstractMech

	/// @brief Abstract motor group class for if you want a custom motor group class
	class AbstractMG : public AbstractComponent {
		private:		
		protected:
		public:
			const pros::MotorGroup mg;

			struct Speeds {
				int fwd = 1000;
				int back = -1000;
			};

			/// @brief Args for abstract motor group object
			/// @param abstractComponentArgs Args for AbstractComponent object
			/// @param ports Vector of ports for motor group
			struct AbstractMGArgs {
				AbstractComponentArgs abstractComponentArgs;
				vector<std::int8_t> ports;
			};

			Speeds speeds = {};

			/// @brief Constructor for abstract motor group object
			/// @param args Args for abstract motor group object (check args struct for more info)
			AbstractMG(AbstractMGArgs args) : 
				AbstractComponent(args.abstractComponentArgs),
				mg(args.ports) {};

			/// @brief Move the motors in the specified direction according to speeds.
			/// @param on Whether to stop or start the motors.
			/// @param directionForward Direction to go.
			void move(bool on, bool directionForward = true) {
				on = canMove(on);

				if (on) {
					if (directionForward) {
						mg.move_velocity(speeds.fwd);
					} else {
						mg.move_velocity(speeds.back);
					}
				} else {
					mg.move_velocity(0);
				}
			}

			virtual bool canMove(bool on) = 0;

			virtual ~AbstractMG() = default;
			
	}; // class AbstractMG

	/// @brief Class for a toggle on the controller
	class MechToggle {
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
				void (AbstractMech::*offFunc)();
				void (AbstractMech::*onFunc)();
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
			struct MechToggleArgs {
				pros::Controller* master;
				AbstractMech* component;
				StaticFuncs staticFuncs;
				StaticOptions st;
			};

			StaticOptions st;

			/// @brief Creates toggle object
			/// @param args Args for toggle object (check args struct for more info)
			MechToggle(MechToggleArgs args) : 
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

	class BiToggle {
		private:

		protected:
		public:
			
	};

	/// @brief Class for driver control
	class Drivetrain : public AbstractComponent {
		public:
			/// @brief Enum for different driver control modes
			enum class DriveControlMode {
				ARCADE

			};
		private:
			pros::MotorGroup left_mg;
			pros::MotorGroup right_mg;

			DriveControlMode driveControlMode;

			std::function<void()> driveControl;

			void bindDriveControl(void (Drivetrain::*driveFunc)()) {
				driveControl = std::bind(driveFunc, this);
			}
		protected:
		public:
			/// @brief Struct for different driver control speeds
			/// @param turnSpeed Speed for turning
			/// @param forwardBackSpeed Speed for forward/backward
			struct DriveControlSpeed {
				float turnSpeed = 2;
				float forwardBackSpeed = 2;
			};

			/// @brief Ports for the drivetrain
			/// @param leftPorts Vector of ports for left motors
			/// @param rightPorts Vector of ports for right motors
			struct DrivetrainPorts {
				vector<std::int8_t> left;
				vector<std::int8_t> right;
			};

			/// @brief Args for drivetrain object
			/// @param abstractComponentArgs Args for AbstractComponent object
			struct DrivetrainArgs {
				AbstractComponentArgs abstractComponentArgs;
				DrivetrainPorts ports;
			};

			DriveControlSpeed driveControlSpeed = {};

			std::int32_t maxRelativeVelocity = 1024;

			Drivetrain(DrivetrainArgs args) : 
				AbstractComponent(args.abstractComponentArgs),
				left_mg(args.ports.left),
				right_mg(args.ports.right) {
					setDriveControlMode();
				};

			void opControl() override {
				driveControl();
			}

			/// @brief Arcade control for drive control (recommended to use opControl instead)
			void arcadeControl() {
				float dir = -1 * master->get_analog(ANALOG_RIGHT_X);    // Gets amount forward/backward from left joystick
				float turn = master->get_analog(ANALOG_LEFT_Y);  // Gets the turn left/right from right joystick

				dir *= driveControlSpeed.forwardBackSpeed;
				turn *= driveControlSpeed.turnSpeed;
				
				int32_t left_voltage = prepareMoveVoltage(dir - turn);                      // Sets left motor voltage
				int32_t right_voltage = prepareMoveVoltage(dir + turn);                     // Sets right motor voltage

				left_mg.move(left_voltage);
				right_mg.move(right_voltage);
			}

			/// @brief Fallback control that DriveControlMode switch statement defaults to.
			void fallbackControl() {
				arcadeControl();
			}

			/// @brief Sets the driver control mode
			/// @param mode Mode to set the driver control to
			void setDriveControlMode(DriveControlMode mode = DriveControlMode::ARCADE) {
				driveControlMode = mode;

				switch (driveControlMode) {
					case DriveControlMode::ARCADE:
						bindDriveControl(&Drivetrain::arcadeControl);
						break;
					default:
						bindDriveControl(&Drivetrain::fallbackControl);
						break;
				}
			}

			/// @brief Gets the driver control mode
			/// @param leftVoltage Voltage for left motor
			/// @param rightVoltage Voltage for right motor
			/// @return Driver control mode
			void moveVelocity(std::int16_t leftVoltage, std::int16_t rightVoltage) {
				left_mg.move_velocity(leftVoltage);
				right_mg.move_velocity(rightVoltage);
			}

			/// @brief Gets the driver control mode
			/// @param pos Position to move to
			void moveRelative(double pos) {
				left_mg.move_relative(pos, maxRelativeVelocity);
				right_mg.move_relative(pos, maxRelativeVelocity);
			}

			/// @brief Gets the left motor group
			pros::MotorGroup& getLeftMotorGroup() {
				return left_mg;
			}

			/// @brief Gets the right motor group
			pros::MotorGroup& getRightMotorGroup() {
				return right_mg;
			}		
	}; // class Drivetrain

	class LiftMech : public AbstractMech {
		private:
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
					actuate(true);
					//pros::lcd::set_text(1, "L1 pressed");
				} else if (master->get_digital(btns.off)) {
					actuate(false);
				}
			}
	}; // class LiftMech

	class MogoMech : public AbstractMech {
		private:
			bool lastPressed = false;

			void processPress() {
				if (!lastPressed) {
					//pros::lcd::set_text(1, "A ENGAGED NOT PRESSED");
					bool engaged = getEngaged();

					if (engaged) {
						actuate(false);
					} else {
						actuate(true);
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

	class Conveyer : public AbstractMG {
		public:
			enum class State {
				FWD,
				BACK,
				OFF
			};

			State state = State::OFF;
			bool lastPressed = false;

			/// @brief Args for pointers required for conveyer object
			/// @param mogoMech Pointer to mogo mech object
			/// @param liftMech Pointer to lift mech object
			struct ReqPointers {
				MogoMech* mogoMech;
				LiftMech* liftMech;
			};

			void processPress(pros::controller_digital_e_t btn) {
				if (!lastPressed) {
					if (state == State::OFF) {
						if (btn == btns.fwd) {
							move(true);
							state = State::FWD;
						} else if (btn == btns.back) {
							move(true, false);
							state = State::BACK;
						}
					} else if (state == State::FWD) {
						if (btn == btns.fwd) {
							move(false);
							state = State::OFF;
						} else if (btn == btns.back) {
							move(true, false);
							state = State::BACK;
						}
					} else if (state == State::BACK) {
						if (btn == btns.fwd) {
							move(true);
							state = State::FWD;
						} else if (btn == btns.back) {
							move(false);
							state = State::OFF;
						}
					}
				}
			}
		private:
			ReqPointers reqPointers;
		protected:
		public:

			/// @brief Args for conveyer object
			/// @param abstractMGArgs Args for AbstractMG object
			/// @param conveyerPorts Vector of ports for conveyer motors
			/// @param mogoMech Pointer to mogo mech object
			struct ConveyerArgs {
				AbstractMGArgs abstractMGArgs;
				ReqPointers reqPointers;
			};

			struct Buttons {
				pros::controller_digital_e_t fwd = pros::E_CONTROLLER_DIGITAL_L1;
				pros::controller_digital_e_t back = pros::E_CONTROLLER_DIGITAL_L2;
			};

			Buttons btns = {};

			Conveyer(ConveyerArgs args) :
				AbstractMG(args.abstractMGArgs), 
				reqPointers(args.reqPointers) {};

			bool canMove(bool on) {
				bool mogoMechMoving = reqPointers.mogoMech->getEngaged();
				bool liftMechMoving = reqPointers.liftMech->getEngaged();

				bool moveConveyer = mogoMechMoving && on || liftMechMoving && on;

				return moveConveyer;
			}

			void opControl() override {
				if (master->get_digital(btns.fwd)) {
					processPress(btns.fwd);
					lastPressed = true;
				} else if (master->get_digital(btns.back)) {
					processPress(btns.back);
					lastPressed = true;
				} else {
					lastPressed = false;
				}
			}
	}; // class Conveyer

	class Intake : public AbstractMG {
		private:
			enum class State {
				FWD,
				BACK,
				OFF
			};

			State state = State::OFF;
			bool lastPressed = false;

			void processPress(pros::controller_digital_e_t btn) {
				if (!lastPressed) {
					if (state == State::OFF) {
						if (btn == btns.fwd) {
							move(true);
							state = State::FWD;
						} else if (btn == btns.back) {
							move(true, false);
							state = State::BACK;
						}
					} else if (state == State::FWD) {
						if (btn == btns.fwd) {
							move(false);
							state = State::OFF;
						} else if (btn == btns.back) {
							move(true, false);
							state = State::BACK;
						}
					} else if (state == State::BACK) {
						if (btn == btns.fwd) {
							move(true);
							state = State::FWD;
						} else if (btn == btns.back) {
							move(false);
							state = State::OFF;
						}
					}
				}
			}
		protected:
		public:
			/// @brief Args for intake object
			/// @param abstractMGArgs Args for AbstractMG object
			/// @param intakePorts Vector of ports for intake motors
			struct IntakeArgs {
				AbstractMGArgs abstractMGArgs;
				vector<std::int8_t> intakePorts;
			};

			struct Speeds {
				int fwd = 1000;
				int back = -1000;
			};

			struct Buttons {
				pros::controller_digital_e_t fwd = pros::E_CONTROLLER_DIGITAL_R1;
				pros::controller_digital_e_t back = pros::E_CONTROLLER_DIGITAL_R2;
			};

			Speeds speeds = {};
			Buttons btns = {};

			/// @brief Constructor for intake object
			/// @param args Args for intake object (see args struct for more info)
			Intake(IntakeArgs args) :
				AbstractMG(args.abstractMGArgs) {}

			bool canMove(bool on) override {
				return on;
			}

			void opControl() override {
				if (master->get_digital(btns.fwd)) {
					processPress(btns.fwd);
					lastPressed = true;
				} else if (master->get_digital(btns.back)) {
					processPress(btns.back);
					lastPressed = true;
				} else {
					lastPressed = false;
				}
			}
	}; // class Intake

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
		protected:
		public:
			/// @brief Args for chassis object
			/// @param dvtArgs Args for drivetrain object
			/// @param mogoMechPort Port for mogo mech
			/// @param liftMechPort Port for lift mech
			/// @param conveyerPorts Vector of ports for conveyer motors
			struct ChassisArgs {
				Drivetrain::DrivetrainPorts dvtPorts;
				char mogoMechPort;
				char liftMechPort;
				vector<std::int8_t> conveyerPorts;
				vector<std::int8_t> intakePorts;
			};

			Drivetrain dvt;

			Auton autonController;

			MogoMech mogoMech;
			LiftMech liftMech;

			Conveyer conveyer;
			Intake intake;

			/// @brief Creates chassis object
			/// @param args Args for chassis object (check args struct for more info)
			Chassis(ChassisArgs args) :  
				dvt({this, args.dvtPorts}),
				mogoMech({this, args.mogoMechPort}), 
				liftMech({this, args.liftMechPort}), 
				conveyer({{this, args.conveyerPorts}, {&mogoMech, &liftMech}}), 
				intake({this, args.intakePorts}),
				autonController(this) {};

			/// @brief Runs the default drive mode specified in opControlMode 
			/// (recommended to be used instead of directly calling the control functions)
			void opControl() override {
				dvt.opControl();
				
				// Run the mainloop for additional components
				// Pneumatics
				mogoMech.opControl();
				liftMech.opControl();

				// Motor groups
				conveyer.opControl();
				intake.opControl();
			}

			/// @brief Auton function for the chassis
			void auton() override {
				autonController.go();
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
	struct MotorBounds {
		static constexpr std::int32_t MOVE_MIN = -127;
		static constexpr std::int32_t MOVE_MAX = 127;
	};

	/// @brief Assert that a value is arithmetic
	/// @param val Value to assert
	template <typename T>
	void assertArithmetic(const T val) {
		static_assert(std::is_arithmetic<T>::value, "Value must be arithmetic");
	}

	/// @brief Clamp a value between a min and max
	/// @param val Value to clamp
	/// @param min Minimum value
	/// @param max Maximum value
	template <typename T>
	T clamp(const T val, const T min, const T max) {
		assertArithmetic(val);

		return std::max(min, std::min(val, max));
	}

	int32_t prepareMoveVoltage(float raw) {
		// Round the number to the nearest integer
		raw = std::round(raw);

		int32_t voltage = static_cast<int32_t>(raw);
		voltage = clamp(voltage, MotorBounds::MOVE_MIN, MotorBounds::MOVE_MAX);

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
	static hyper::Chassis defaultChassis({
		{LEFT_DRIVE_PORTS, RIGHT_DRIVE_PORTS}, 
	MOGO_MECH_PORT, LIFT_MECH_PORT, CONVEYER_PORTS, INTAKE_PORTS});
	
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
	pros::lcd::set_text(0, "> 1408Hyper mainControl ready");

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
