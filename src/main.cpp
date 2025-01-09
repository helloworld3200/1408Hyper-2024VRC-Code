// includes/usings are all in main.h
#include "main.h"

// nothing to see here, move along
																																																																																																									#define _HYPER_UNLEASH_HELL delete this, *(reinterpret_cast<int*>(this) + 1) = 0xDEADBEEF;
// uses ISO/C++20 standard
// added libraries/includes:
// fmt (header-only)

// ending in -mech classes are for pneumatics

// currently using legacy toggles (not using MechToggle class):
// mogomech - maybe try to upgrade to MechToggle?

// CONSIDER odom?

// WARNING: do NOT just put "Args" as the name of an args struct in any class
// instead, put the class name in front of it (e.g. DrivetrainArgs) for CLARITY
// in derived functions & then for factories just do using e.g. using ArgsType = DrivetrainArgs;

// TODO: seperate PID functions into a separate class for cleanliness

/// @brief Hyper namespace for all custom classes and functions
namespace hyper {
	// Function declarations
	template <typename T>
	string vectorToString(vector<T>& vec, string delimiter = ", ");

	std::int32_t prepareMoveVoltage(float raw);

	template <typename T>
	bool isNumBetween(T num, T min, T max);

	template <typename T>
	T normaliseAngle(T angle);

	template <typename T>
	T naiveNormaliseAngle(T angle);

	template <typename T>
	vector<T> getAllValues() {}

	template <typename E, typename V>
	void fillMapWithEnum(map<E, V>& map) {}

	// Structs

	/// @brief Struct for motor group buttons (manual control)
	/// @param fwd Button for forward
	/// @param back Button for backward

	struct Buttons {
		pros::controller_digital_e_t fwd;
		pros::controller_digital_e_t back;
	};

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
			virtual void skillsPrep() = 0;
			virtual void skillsAuton() = 0;
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

			/// @brief Creates AbstractComponent object (WARNING: Use ComponentArgsFactory to create args instead of directly)
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

			pros::Controller& getMaster() {
				return *master;
			}

			virtual void opControl() = 0;

			virtual ~AbstractComponent() = default;
	}; // class ChassisComponent

	/// @brief Class which instantiates component arguments easily
	class ComponentArgsFactory {
		private:
		protected:
		public:
			// !!!!!!!!!!!!!!!!!!!!!!!!!!!!
			// ACA = AbstractComponentArgs
			// !!!!!!!!!!!!!!!!!!!!!!!!!!!!

			AbstractComponent::AbstractComponentArgs aca;

			/// @brief Args for component factory object
			/// @param aca Args to instantiate any abstract component
			struct ComponentArgsFactoryArgs {
				AbstractComponent::AbstractComponentArgs aca;
			};

			/// @brief Creates component factory object
			/// @param args Args for component factory object (check args struct for more info)
			ComponentArgsFactory(ComponentArgsFactoryArgs args) : 
				aca(args.aca) {};

			/// @brief Manages safe creation of args for a specific component
			template <typename ComponentType, typename... Args>
			typename ComponentType::ArgsType create(Args&&... args) {
				return {aca, std::forward<Args>(args)...};
			}
	}; // class ComponentFactory

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
			const pros::MotorGroup mg;
		public:
			struct Speeds {
				int fwd = 10000;
				int back = -10000;
			};

			/// @brief Args for abstract motor group object
			/// @param abstractComponentArgs Args for AbstractComponent object
			/// @param ports Vector of ports for motor group
			struct AbstractMGArgs {
				AbstractComponentArgs abstractComponentArgs;
				MGPorts ports;
			};

			Speeds speeds = {};
			bool outputSpeeds;

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
						if (outputSpeeds) {
							pros::lcd::print(2, "motor going!!");
						}
					} else {
						mg.move_velocity(speeds.back);
						if (outputSpeeds) {
							pros::lcd::print(2, "motor not going :(");
						}
					}
				} else {
					mg.move_velocity(0);
				}
			}

			virtual bool canMove(bool on) = 0;

			virtual ~AbstractMG() = default;
			
	}; // class AbstractMG

	/// @brief Class which manages button presses (will run function on up, down and hold states of given button)
	class BtnManager : public AbstractComponent {
		private:
			bool lastPressed = false;

			void handleBtnPressed() {
				if (lastPressed) {
					for (VoidFunc& func: actionInfo.holdFuncs) {
						func();
					}
				} else {
					for (VoidFunc& func: actionInfo.downFuncs) {
						func();
					}
				}
			}
		protected:
		public:
			/// @brief Struct for action info for button manager object
			/// @param upFuncs Functions that are run once when up state is reached
			/// @param downFuncs Functions that are run once when down state is reached
			/// @param holdFuncs Functions to continuously run on hold state
			/// @param btn Button to manage
			struct ActionInfo {
				pros::controller_digital_e_t btn;
				VoidFuncVector downFuncs = {};
				VoidFuncVector upFuncs = {};
				VoidFuncVector holdFuncs = {};
			};

			ActionInfo actionInfo;

			/// @brief Args for button manager object
			/// @param abstractComponentArgs Args for AbstractComponent object
			/// @param actionInfo Action info for button manager object
			struct BtnManagerArgs {
				AbstractComponentArgs abstractComponentArgs;
				ActionInfo actionInfo;
			};

			/// @brief Creates button manager object
			/// @param args Args for button manager object (check args struct for more info)
			BtnManager(BtnManagerArgs args) : 
				AbstractComponent(args.abstractComponentArgs),
				actionInfo(args.actionInfo) {};

			void opControl() override {
				bool btnPressed = master->get_digital(actionInfo.btn);
				
				// down: !lastPressed && btnPressed
				// up: lastPressed && !btnPressed
				// hold: lastPressed && btnPressed

				if (btnPressed) {
					handleBtnPressed();
				} else if (lastPressed) {
					for (VoidFunc& func: actionInfo.upFuncs) {
						func();
					}
				}

				lastPressed = btnPressed;
			}

			bool getLastPressed() {
				return lastPressed;
			}
	};

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

	/// @brief Class for a toggle on the controller
	class BiToggle { // Don't need to derive from AbstractComponent because no need for Chassis pointer
		public:
			enum class State {
				OFF,
				FWD,
				BACK
			};
		private:
			AbstractMG* component;

			pros::Controller* master;
			
			State state = State::OFF;
			bool isNewPress = true;

			void moveState(State target) {
				if (!isNewPress) {
					return;
				}

				switch (target) {
					case State::OFF:
						component->move(false);
						break;
					case State::FWD:
						component->move(true);
						break;
					case State::BACK:
						component->move(true, false);
						break;
				}
				
				state = target;
			}

			void handleFwdBtn() {
				if (state == State::FWD) {
					moveState(State::OFF);
					pros::lcd::print(1, "Fwd pressed AND GOING OFF");
				} else {
					moveState(State::FWD);
					pros::lcd::print(1, "Fwd pressed AND GOING FWD");
				}
			}

			void handleBackBtn() {
				if (state == State::BACK) {
					moveState(State::OFF);
					pros::lcd::print(1, "Back pressed AND GOING OFF");
				} else {
					moveState(State::BACK);
					pros::lcd::print(1, "Back pressed AND GOING BACK");
				}
			}
		protected:
		public:
			/// @brief Struct for buttons for BiToggle object
			/// @param fwd Button for forward
			/// @param back Button for backward
			struct Buttons {
				pros::controller_digital_e_t fwd;
				pros::controller_digital_e_t back;
			};

			/// @brief Args for BiToggle object
			/// @param component Component to toggle
			/// @param btns Buttons for toggle
			struct BiToggleArgs {
				AbstractMG* component;
				Buttons btns;
			};

			Buttons btns;

			/// @brief Creates BiToggle object
			/// @param args Args for BiToggle object (check args struct for more info)
			BiToggle(BiToggleArgs args) : 
				component(args.component),
				btns(args.btns),
				master(&args.component->getMaster()) {};

			void opControl() {
				bool fwdPressed = master->get_digital(btns.fwd);
				bool backPressed = master->get_digital(btns.back);

				pros::lcd::print(3, ("FWD: " + std::to_string(fwdPressed)).c_str());
				pros::lcd::print(4, ("BACK: " + std::to_string(backPressed)).c_str());

				if (fwdPressed && backPressed) {
					// Don't do anything if both are pressed
					// TODO: test whether the return works
					// because we need it for backwards motor movement
					return;
				}

				if (fwdPressed) {
					pros::lcd::print(2, "Begin CONVEYER FWD");
					handleFwdBtn();
					isNewPress = false;
					return;
				}

				if (backPressed) {
					pros::lcd::print(2, "Begin CONVEYER BACK");
					handleBackBtn();
					isNewPress = false;
					return;
				}

				isNewPress = true;
			}

			void setState(State target) {
				state = target;
			}

			State getState() {
				return state;
			}
	}; // class BiToggle

	/// @brief Class for driver control
	class Drivetrain : public AbstractComponent {
		public:
			/// @brief Enum for different driver control modes
			enum class DriveControlMode {
				ARCADE

			};
		private:
			// Coefficients for turning in driver control
			struct TurnCoefficients {
				float left;
				float right;
			};

			pros::MotorGroup left_mg;
			pros::MotorGroup right_mg;

			pros::IMU imu;

			DriveControlMode driveControlMode;

			std::function<void()> driveControl;

			void bindDriveControl(void (Drivetrain::*driveFunc)()) {
				driveControl = std::bind(driveFunc, this);
			}
		protected:
		public:
			/// @brief Struct for different driver control speeds
			/// @param turnSpeed Multiplier for only turning
			/// @param forwardBackSpeed Multiplier for only forward/backward
			/// @param arcSpeed Multiplier of opposite turn for when turning and moving laterally at the same time
			// (higher value means less lateral movement)
			struct DriveControlSpeed {
				private:
					float forwardBackSpeed;
					float maxLateral;
				public:
					static constexpr float controllerMax = 127;

					float turnSpeed;
					float arcSpeed;

					/// @brief Sets the forward/backward speed
					/// @param speed Speed to set the forward/backward speed to
					// (Also prepares maxLateral for arc movement)
					void setForwardBackSpeed(float speed, float maxTolerance = 1) {
						forwardBackSpeed = speed;
						maxLateral = speed * controllerMax + maxTolerance;
					}

					/// @brief Gets the forward/backward speed
					/// @return Forward/backward speed
					float getForwardBackSpeed() {
						return forwardBackSpeed;
					}

					/// @brief Gets the max lateral movement
					/// @return Max lateral movement
					float getMaxLateral() {
						return maxLateral;
					}

					// lower arc speed is lower turning

					DriveControlSpeed(float turnSpeed = 1, float forwardBackSpeed = 2, float arcSpeed = 0.5) :
						turnSpeed(turnSpeed), 
						arcSpeed(arcSpeed) {
							setForwardBackSpeed(forwardBackSpeed);
						}
			};


			/// @brief Ports for the drivetrain
			/// @param leftPorts Vector of ports for left motors
			/// @param rightPorts Vector of ports for right motors
			struct DrivetrainPorts {
				vector<std::int8_t> left;
				vector<std::int8_t> right;
				std::int8_t imuPort;
			};

			/// @brief Args for drivetrain object
			/// @param abstractComponentArgs Args for AbstractComponent object
			struct DrivetrainArgs {
				AbstractComponentArgs abstractComponentArgs;
				DrivetrainPorts ports;
			};

			using ArgsType = DrivetrainArgs;

			/// @brief Struct for PID options (self-explanatory - timeLimit in MS)
			struct PIDOptions {
				double kP;
				double kI;
				double kD;
				double errorThreshold;
				float timeLimit;
			};

			DriveControlSpeed driveControlSpeed = {};

			bool preventBackMove = false;

			std::int32_t defaultMoveVelocity = 1024;
			std::int8_t maxRelativeError = 5;

			std::int16_t maxTurnVelocity = 60;
			float minTurnThreshold = 5;

			float relativeMovementCoefficient = 14.2857;
			float voltMovementCoefficient = 1;

			float maxVoltage = 12000;

			//double inchesPerTick = 0.025525;
			double inchesPerTick = 0.0127625;

			uint32_t moveDelayMs = 2;

			int pidInvertTurn = 1;

			float arcDeadband = 5;

			Drivetrain(DrivetrainArgs args) : 
				AbstractComponent(args.abstractComponentArgs),
				left_mg(args.ports.left),
				right_mg(args.ports.right),
				imu(args.ports.imuPort) {
					setDriveControlMode();
					calibrateIMU();
				};
		private:
			void prepareArcadeLateral(float& lateral) {
				// Change to negative to invert
				lateral *= -1;

				// Clamp the range to above 0 only to remove back movement
				if (preventBackMove && (lateral < 0)) {
					lateral = 0;
				}

				lateral *= driveControlSpeed.getForwardBackSpeed();
			}

			// Calculate the movement of the robot when turning and moving laterally at the same time
			void calculateArcMovement(TurnCoefficients& turnCoeffs, float lateral, float turn, float maxLateralTolerance = 1) {
				if (std::fabs(lateral) < arcDeadband) {
					return;
				}

				// 0-1 range of percentage of lateral movement against max possible lateral movement
				float lateralCompensation = lateral / driveControlSpeed.getMaxLateral();
				// Decrease the turn speed when moving laterally (higher turn should be higher turnDecrease)
				float turnDecrease = turn * (1 - (lateralCompensation * driveControlSpeed.arcSpeed));

				if (turn > 0) { // Turning to right so we decrease the left MG
					turnCoeffs.left -= turnDecrease;
				} else { // Turning to left so we decrease the right MG
					turnCoeffs.right -= turnDecrease;
				}

				pros::lcd::print(4, ("Left Turn Coef: " + std::to_string(turnCoeffs.left)).c_str());
				pros::lcd::print(5, ("Right Turn Coef: " + std::to_string(turnCoeffs.right)).c_str());
			}

			TurnCoefficients calculateArcadeTurns(float turn, float lateral) {
				turn *= 1;
				turn *= driveControlSpeed.turnSpeed;

				TurnCoefficients turnCoeffs = {turn, turn};

				// Allow for arc movement
				calculateArcMovement(turnCoeffs, lateral, turn);

				return turnCoeffs;
			}
		public:
			void opControl() override {
				driveControl();
			}

			/// @brief Arcade control for drive control (recommended to use opControl instead)
			void arcadeControl() {
				float lateral = master->get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
				float turn = master->get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick

				prepareArcadeLateral(lateral);

				TurnCoefficients turnCoeffs = calculateArcadeTurns(turn, lateral);

				// Ensure voltages are within correct ranges
				std::int32_t left_voltage = prepareMoveVoltage(lateral - turnCoeffs.left);
				std::int32_t right_voltage = prepareMoveVoltage(lateral + turnCoeffs.right);

				//pros::lcd::print(4, ("LEFT/RIGHT: " + std::to_string(master->get_analog(ANALOG_LEFT_Y)) + ", " + std::to_string(master->get_analog(ANALOG_RIGHT_X)))).c_str();

				left_mg.move(left_voltage);
				right_mg.move(right_voltage);
			}

			/// @brief Sets the brake mode for each motor group
			/// @param mode Brake mode to set the motors toS
			void setBrakeModes(pros::motor_brake_mode_e_t mode) {
				left_mg.set_brake_mode(mode);
				right_mg.set_brake_mode(mode);
			}

			/// @brief Fallback control that DriveControlMode switch statement defaults to.
			void fallbackControl() {
				arcadeControl();
			}

			/// @brief Calibrates the IMU
			void calibrateIMU(bool blocking = true) {
				imu.reset(blocking);
				imu.tare();
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

			/// @brief Moves the motors at a specific voltage
			/// @param leftVoltage Voltage for left motor
			/// @param rightVoltage Voltage for right motor
			void moveVoltage(std::int16_t leftVoltage, std::int16_t rightVoltage) {
				left_mg.move_voltage(leftVoltage);
				right_mg.move_voltage(rightVoltage);
				pros::lcd::print(0, ("Left Voltage: " + std::to_string(leftVoltage)).c_str());
				pros::lcd::print(1, ("Right Voltage: " + std::to_string(rightVoltage)).c_str());
			}

			/// @brief Moves the motors at a single voltage
			/// @param voltage Voltage to move the motors at
			void moveSingleVoltage(std::int16_t voltage) {
				moveVoltage(voltage, voltage);
			}

			/// @brief Sets movement velocity
			/// @param leftVoltage Voltage for left motor
			/// @param rightVoltage Voltage for right motor
			void moveVelocity(std::int16_t leftVoltage, std::int16_t rightVoltage) {
				left_mg.move_velocity(leftVoltage);
				right_mg.move_velocity(rightVoltage);
			}

			/// @brief Moves the motors at a single velocity
			/// @param voltage Voltage to move the motors at
			void moveSingleVelocity(std::int16_t voltage) {
				moveVelocity(voltage, voltage);
			}

			/// @brief Stops moving the motors
			void moveStop() {
				moveSingleVelocity(0);
			}

			/// @brief Tares the motors
			void tareMotors() {
				left_mg.tare_position();
				right_mg.tare_position();
			}

			/// @brief Move to relative position
			/// @param pos Position to move to in CM
			void moveRelPos(double pos) {
				tareMotors();

				pos *= relativeMovementCoefficient;

				left_mg.move_relative(pos, defaultMoveVelocity);
				right_mg.move_relative(pos, defaultMoveVelocity);

				double lowerError = pos - maxRelativeError;

				while ((
					!(left_mg.get_position() > lowerError)
				) && (
					!(right_mg.get_position() > lowerError)
				)) {
					pros::delay(moveDelayMs);
				}

				moveStop();
			}

			/// @brief Turn to a specific angle
			/// @param angle Angle to turn to
			void turnTo(double angle) {
				imu.tare();

				double currentHeading = imu.get_heading();
				double angleDifference = normaliseAngle(angle - currentHeading);

				std::int16_t turnDirection = (angleDifference > 0) ? maxTurnVelocity : -maxTurnVelocity;
				//std::int16_t turnDirection = maxTurnVelocity;
				
				left_mg.move_velocity(turnDirection);
				right_mg.move_velocity(-turnDirection);

				while (true) {
					currentHeading = imu.get_heading();
					angleDifference = normaliseAngle(angle - currentHeading);
					
					if (std::fabs(angleDifference) <= minTurnThreshold) {
						break;
					}
					
					pros::lcd::set_text(2, "Current heading: " + std::to_string(currentHeading));
					pros::delay(moveDelayMs);
				}

				moveStop();
			}

			/// @brief Turn to a specific angle with a delay
			/// @param angle Angle to turn to
			/// @param delayMs Delay in milliseconds
			void turnDelay(bool direction, std::uint32_t delayMs) {
				std::int16_t turnDirection = (direction) ? maxTurnVelocity : -maxTurnVelocity;

				left_mg.move_velocity(turnDirection);
				right_mg.move_velocity(-turnDirection);

				pros::delay(delayMs);

				moveStop();
			}

			/// @brief Move forward for a certain number of milliseconds
			/// @param delayMs Number of milliseconds to move forward
			/// @param left Whether to move the left motor
			/// @param right Whether to move the right motor
			void moveDelay(std::uint32_t delayMs, bool forward = true) {
				if (forward) {
					moveSingleVelocity(-defaultMoveVelocity);
				} else {
					moveSingleVelocity(defaultMoveVelocity);
				}

				pros::delay(delayMs);
				moveStop();
			}

			double getHeading() {
				return imu.get_heading();
			}

			// TODO: Generic PID function that we can apply to PIDTurn and PIDMove
			// maybe make a class for this? if it gets too complicated
			// but that would also require refactoring Drivetrain to have an AbstractDrivetrain
			// parent to avoid cyclic dependencies

			// WARNING: do NOT use relativeMovementCoefficient for PID functions
			// as this does not account for acceleration/deceleration
			// it's only for simple movement (phased out by PID & PIDOptions struct)

			/// @brief Turn to a specific angle using PID
			/// @param angle Angle to move to (PASS IN THE RANGE OF -180 TO 180 for left and right)
			// TODO: Tuning required
			void PIDTurn(double angle, PIDOptions options = {
				0.3, 0.0, 0.7, 1, 6000
			}) {
				imu.tare();
				angle = naiveNormaliseAngle(angle);

				angle *= pidInvertTurn;

				angle /= 1;

				bool anglePositive = angle > 0;
				bool turn180 = false;

				// IMU already tared so we don't need to get the current heading
				float error = angle;
				float lastError = 0;
				float derivative = 0;
				float integral = 0;

				float out = 0;
				float trueHeading = 0;

				float maxThreshold = 180 - options.errorThreshold;

				float maxCycles = options.timeLimit / moveDelayMs;
				float cycles = 0;

				if (std::fabs(angle) >= 180) {
					turn180 = true;
				}

				pros::lcd::print(3, "PIDTurn Start");

				// with turning you just wanna move the other MG at negative of the MG of the direction
				// which u wanna turn to

				while (true) {
					trueHeading = std::fmod((imu.get_heading() + 180), 360) - 180;
					error = angle - trueHeading;

					integral += error;
					// Anti windup
					if (std::fabs(error) < options.errorThreshold) {
						integral = 0;
					}

					derivative = error - lastError;
					out = (options.kP * error) + (options.kI * integral) + (options.kD * derivative);
					lastError = error;

					out *= 1000; // convert to mV
					out = std::clamp(out, -maxVoltage, maxVoltage);
					moveVoltage(-out, out);

					pros::lcd::print(5, ("PIDTurn Out: " + std::to_string(out)).c_str());
					pros::lcd::print(7, ("PIDTurn Error: " + std::to_string(error)).c_str());
					pros::lcd::print(6, ("PIDTurn True Heading: " + std::to_string(imu.get_heading())).c_str());

					if (std::fabs(error) <= options.errorThreshold) {
						break;
					}

					// 180 degree turning
					if (std::fabs(trueHeading) >= maxThreshold) {
						break;
					}

					// TODO: refactor checks in prod
					if (std::fabs(out) < 100) {
						pros::lcd::print(4, "PIDTurn Out too low");
					}

					if (cycles >= maxCycles) {
						pros::lcd::print(4, "PIDTurn Time limit reached");
						break;
					}

					pros::delay(moveDelayMs);
					cycles++;
				}

				pros::lcd::print(2, "PIDTurn End");
				moveStop();
			}

			// think about arc motion, odometry, etc.
			// the key thing is PID.
			// TUNING REQUIRED!!!

			/// @brief Move to a specific position using PID
			/// @param pos Position to move to in inches (use negative for backward)
			// TODO: Tuning required
			void PIDMove(double pos, PIDOptions options = {
				0.15, 0.0, 0.6, 3, 6000
			}) {
				// TODO: Consider adding odometry wheels as the current motor encoders
				// can be unreliable for long distances or just dont tare the motors
				tareMotors();

				pos /= inchesPerTick;
				pos *= -1;

				float error = pos;
				float motorPos = 0;
				float lastError = 0;
				float derivative = 0;
				float integral = 0;
				float out = 0;

				float maxCycles = options.timeLimit / moveDelayMs;
				float cycles = 0;

				// with moving you just wanna move both MGs at the same speed

				while (true) {
					// get avg error
					motorPos = (left_mg.get_position() + right_mg.get_position()) / 2;
					error = pos - motorPos;

					integral += error;
					// Anti windup
					if (std::fabs(error) < options.errorThreshold) {
						integral = 0;
					}

					derivative = error - lastError;
					out = (options.kP * error) + (options.kI * integral) + (options.kD * derivative);
					lastError = error;

					out *= 1000; // convert to mV
					out = std::clamp(out, -maxVoltage, maxVoltage);
					moveSingleVoltage(out);

					if (std::fabs(error) <= options.errorThreshold) {
						break;
					}

					pros::lcd::print(4, ("PIDMove Motor Pos: " + std::to_string(motorPos)).c_str());
					pros::lcd::print(5, ("PIDMove Out: " + std::to_string(out)).c_str());
					pros::lcd::print(7, ("PIDMove Error: " + std::to_string(error)).c_str());

					if (cycles >= maxCycles) {
						pros::lcd::print(4, "PIDMove Time limit reached");
						break;
					}

					pros::delay(moveDelayMs);
					cycles++;
				}

				moveStop();
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

	class MogoMech : public AbstractMech {
		private:
			bool lastPressed = false;

			void processPress() {
				if (!lastPressed) {
					//pros::lcd::set_text(1, "A ENGAGED NOT PRESSED");
					bool engaged = getEngaged();

					if (engaged) {
						actuate(false);
						master->print(0, 0, "Mogo engaged: YES");
					} else {
						actuate(true);
						master->print(0, 0, "Mogo engaged: NO");
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

			using ArgsType = MogoMechArgs;

			/// @brief Creates mogo mech object
			/// @param args Args for MogoMech object (check args struct for more info)
			MogoMech(MogoMechArgs args) : 
				AbstractMech(args.abstractMechArgs) {
					actuate(true);
				};

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
		private:
		protected:
		public:
			BiToggle toggle;

			/// @brief Args for conveyer object
			/// @param abstractMGArgs Args for AbstractMG object
			/// @param conveyerPorts Vector of ports for conveyer motors
			/// @param mogoMech Pointer to mogo mech object
			struct ConveyerArgs {
				AbstractMGArgs abstractMGArgs;
			};

			using ArgsType = ConveyerArgs;

			Conveyer(ConveyerArgs args) :
				AbstractMG(args.abstractMGArgs), 
				toggle({this, {
					pros::E_CONTROLLER_DIGITAL_R2,
					pros::E_CONTROLLER_DIGITAL_R1
				}}) {
					speeds = {10000, -10000};
					outputSpeeds = true;
					mg.set_gearing_all(pros::motor_gearset_e_t::E_MOTOR_GEAR_RED);
				};

			bool canMove(bool on) override {
				/* Dont need this for now because lift mech doesn't exist
				bool mogoMechMoving = reqPointers.mogoMech->getEngaged();
				bool liftMechMoving = reqPointers.liftMech->getEngaged();

				bool moveConveyer = (mogoMechMoving && on) || (liftMechMoving && on);

				// DISABLE THIS FOR NOW BECAUSE WE DONT HAVE A LIFT MECH
				return moveConveyer;*/

				return on;
			}

			void opControl() override {
				toggle.opControl();
			}
	}; // class Conveyer

	class Intake : public AbstractMG {
		private:
			BiToggle toggle;
		protected:
		public:
			/// @brief Args for intake object
			/// @param abstractMGArgs Args for AbstractMG object
			/// @param intakePorts Vector of ports for intake motors
			struct IntakeArgs {
				AbstractMGArgs abstractMGArgs;
				vector<std::int8_t> intakePorts;
			};

			using ArgsType = IntakeArgs;

			/// @brief Constructor for intake object
			/// @param args Args for intake object (see args struct for more info)
			Intake(IntakeArgs args) :
				AbstractMG(args.abstractMGArgs),
				toggle({this, {
					pros::E_CONTROLLER_DIGITAL_L1,
					pros::E_CONTROLLER_DIGITAL_L2
				}}) {}

			bool canMove(bool on) override {
				return on;
			}

			void opControl() override {
				toggle.opControl();
			}
	}; // class Intake

	class MogoStopper : public AbstractComponent {
		private:
			pros::Optical sensor;
			MogoMech* mogoMech;
			BtnManager btn;

			bool doActuate = false;			
		protected:
		public:
			pros::c::optical_rgb_s_t mogoRGB = {233, 255, 8};
			float tolerance = 5;
		private:
			bool channelWithinTolerance(const float& channel, const float& target) {
				return std::fabs(channel - target) <= tolerance;
			}

			bool rgbIsMogo(const pros::c::optical_rgb_s_t& rgb) {
				bool redWithinTolerance = channelWithinTolerance(rgb.red, mogoRGB.red);
				bool greenWithinTolerance = channelWithinTolerance(rgb.green, mogoRGB.green);
				bool blueWithinTolerance = channelWithinTolerance(rgb.blue, mogoRGB.blue);

				bool withinTolerance = redWithinTolerance && greenWithinTolerance && blueWithinTolerance;

				return withinTolerance;
			}
		public:

			/// @brief Args for mogo stopper object
			/// @param abstractComponentArgs Args for AbstractComponent object
			/// @param stopperPort Port for stopper motor
			struct MogoStopperArgs {
				AbstractComponentArgs abstractComponentArgs;
				std::int8_t sensorPort;
				MogoMech* mogoMech;
			};

			using ArgsType = MogoStopperArgs;

			void toggleActuate() {
				doActuate = !doActuate;
			}

			/// @brief Constructor for mogo stopper object
			/// @param args Args for mogo stopper object (see args struct for more info)
			MogoStopper(MogoStopperArgs args) : 
				AbstractComponent(args.abstractComponentArgs),
				sensor(args.sensorPort),
				mogoMech(args.mogoMech),
				btn({args.abstractComponentArgs, {
					pros::E_CONTROLLER_DIGITAL_B, {std::bind(&MogoStopper::toggleActuate, this)}, {}, {}
				}}) {};
			
			///#DFFF08 - Mobile goal hex code
			///R-233 G-255 B-8 Mobile goal RGB

			void opControl() override {
				if (!doActuate) {
					return;
				}

				pros::c::optical_rgb_s_t rgb = sensor.get_rgb();

				if (rgbIsMogo(rgb)) {
					mogoMech->actuate(true);
				}
			}
	}; // class MogoStopper

	/// @brief Class which manages the Lady Brown mechanism
	class LadyBrown : public AbstractMG {
		private:
			int currentTarget = 0;
		protected:
		public:
			/// @brief Args for Lady Brown object
			/// @param abstractMGArgs Args for AbstractMG object
			/// @param ladyBrownPorts Vector of ports for Lady Brown motors
			struct LadyBrownArgs {
				AbstractMGArgs abstractMGArgs;
				MGPorts ladyBrownPorts;
			};

			using ArgsType = LadyBrownArgs;

			// Target position to move to (start, halfway, end)
			vector<double> targets = {0, 180, -1};

			BtnManager upBtn;
			bool atManualControl = false;

			double limit = 10000;

			Buttons manualBtns = {
				pros::E_CONTROLLER_DIGITAL_UP,
				pros::E_CONTROLLER_DIGITAL_DOWN
			};

			/// @brief Changes the target by the amount specified by the change parameter
			/// @param change Amount to change the target by
			void changeTarget(int change) {
				int limit = targets.size() - 1;
				currentTarget += change;
				currentTarget = std::clamp(currentTarget, 0, limit);
			}

			void incrementTarget() {
				changeTarget(1);
			}

			void decrementTarget() {
				changeTarget(-1);
			}
		private:
			void manualControl() {
				//bool belowLimit = mg.get_position() < limit;
				bool belowLimit = true;

				if (master->get_digital(manualBtns.fwd) && belowLimit) {
					move(true);
				} else if (master->get_digital(manualBtns.back)) {
					move(true, false);
				} else {
					move(false);
				}
			}

			void upBtnControl() {
				incrementTarget();

				if (targets[currentTarget] == -1) {
					atManualControl = true;
				}
			}
		public:
			/// @brief Constructor for Lady Brown object
			/// @param args Args for Lady Brown object (see args struct for more info)
			LadyBrown(LadyBrownArgs args) : 
				AbstractMG(args.abstractMGArgs),
				upBtn({args.abstractMGArgs.abstractComponentArgs, {
					pros::E_CONTROLLER_DIGITAL_UP, {std::bind(&LadyBrown::upBtnControl, this)}, {}, {} 
				}}) {
					mg.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
				};

			bool canMove(bool on) override {
				return on;
			}

			void opControl() override {
				upBtn.opControl();

				// Auto control if not manual controol
				if (atManualControl) {
					manualControl();
				} else {
					mg.move_absolute(targets[currentTarget], speeds.fwd);
				}
				// TODO: implement moving the lady brown to targets with PID
			}
	}; // class LadyBrown

	class Doinker : public AbstractMech {
		private:
		protected:
		public:
			/// @brief Args for doinker object
			/// @param abstractMechArgs Args for AbstractMech object
			struct DoinkerArgs {
				AbstractMechArgs abstractMechArgs;
			};

			using ArgsType = DoinkerArgs;

			BtnManager actuateBtn;

			void handleBtn() {
				actuate(!getEngaged());
				//master->set_text(1, 0, "Doinker state: " + std::to_string(state));
			}

			/// @brief Constructor for doinker object
			/// @param args Args for doinker object (see args struct for more info)
			Doinker(DoinkerArgs args) : 
				AbstractMech(args.abstractMechArgs),
				actuateBtn({args.abstractMechArgs.abstractComponentArgs, {
					pros::E_CONTROLLER_DIGITAL_X, {std::bind(&Doinker::handleBtn, this)}, {}, {}
				}
				}) {};

			void opControl() override {
				actuateBtn.opControl();
			}
	};

	/// @brief Class which manages all components
	class ComponentManager : public AbstractComponent {
		private:
		protected:
		public:
			ComponentArgsFactory factory;

			Drivetrain dvt;

			MogoMech mogoMech;
			Doinker doinker;

			Conveyer conveyer;
			LadyBrown ladyBrown;

			// All components are stored in this vector
			vector<AbstractComponent*> components;

			/// @brief Args for component manager object passed to the chassis, such as ports
			/// @param dvtArgs Args for drivetrain object
			/// @param mogoMechArgs Args for mogo mech object
			/// @param conveyerArgs Args for conveyer object
			/// @param intakeArgs Args for intake object
			struct ComponentManagerUserArgs {
				Drivetrain::DrivetrainPorts dvtPorts;
				char mogoMechPort;
				char doinkerPort;
				MGPorts conveyerPorts;
				MGPorts ladyBrownPorts;
			};

			/// @brief Args for component manager object
			/// @param abstractComponentArgs Args for AbstractComponent object
			/// @param user Args for component manager object passed to the chassis
			struct ComponentManagerArgs {
				AbstractComponentArgs abstractComponentArgs;
				ComponentManagerUserArgs user;
			};

			/// @brief Constructor for component manager object
			/// @param args Args for component manager object (see args struct for more info)
			ComponentManager(ComponentManagerArgs args) : 
				AbstractComponent(args.abstractComponentArgs),
				factory({args.abstractComponentArgs}),

				dvt(factory.create<Drivetrain>(args.user.dvtPorts)),
				mogoMech(factory.create<MogoMech>(args.user.mogoMechPort)),
				conveyer(factory.create<Conveyer>(args.user.conveyerPorts)),
				ladyBrown(factory.create<LadyBrown>(args.user.ladyBrownPorts)),
				doinker(factory.create<Doinker>(args.user.doinkerPort)) {
					// Add component pointers to vector
					// MUST BE DONE AFTER INITIALISATION not BEFORE because of pointer issues
					components = {
						&dvt,
						&mogoMech,
						&conveyer,
						&ladyBrown,
						&doinker
					};
				};

			// Nice and simple :) definitely better than having to call each component individually
			void opControl() override {
				for (AbstractComponent* component : components) {
					component->opControl();
				}
			}
	}; // class ComponentManager

	/// @brief Abstract class for auton e.g. match or skills autonomous
	class AbstractAuton {
		private:
		protected:
			ComponentManager* cm;
		public:
			/// @brief Args for auton object
			/// @param cm Component manager object
			struct AutonArgs {
				ComponentManager* cm;
			};

			/// @brief Creates auton object
			/// @param args Args for auton object (check args struct for more info)
			AbstractAuton(AutonArgs args) : 
				cm(args.cm) {};

			/// @brief Runs the auton
			virtual void run() = 0;

			virtual ~AbstractAuton() = default;
	}; // class AbstractAuton

	class MatchAuton : public AbstractAuton {
		private:
			void defaultAuton() {
				// destruction 100

				/*//cm->dvt.moveRelPos(300);
				//
				cm->dvt.turnDelay(true, 600);
				cm->dvt.moveRelPos(100);
				cm->dvt.turnDelay(false, 400);
				//cm->dvt.moveRelPos(150);
				cm->dvt.turnDelay(true, 300);*/

				/*cm->intake.move(true, false);
				cm->conveyer.move(true);*/

				// Because auton is only 15 secs no need to divide into sectors
				// Move and collect first rings/discombobulate first
				//cm->intake.move(true);
				cm->dvt.turnDelay(true, 600);
				//pros::delay(MAINLOOP_DELAY_TIME_MS);
				cm->dvt.moveRelPos(50);

				// Get the far ring and turn back onto main path
				// (no longer necessarily needed because we start with 1 ring already in the robot)
				cm->dvt.turnDelay(true, 330);
				cm->dvt.moveRelPos(105);
				//pros::delay(MAINLOOP_DELAY_TIME_MS);
				//cm->dvt.turnDelay(false, 1.5);

				// Get other stack knocked over
				// optional: increase speed to cm->intake if we have no harvester
				//cm->dvt.moveRelPos(130);
				//cm->dvt.moveDelay(600, false);
				cm->dvt.turnDelay(false, 450);
				//pros::delay(MAINLOOP_DELAY_TIME_MS);
				cm->dvt.moveRelPos(160);
				
				// Turn into high wall stake & deposit
				cm->dvt.turnDelay(false, 870);
				cm->dvt.moveDelay(800, false);
				//cm->intake.move(false);
				//liftMech.actuate(true);
				//pros::delay(MAINLOOP_DELAY_TIME_MS);

				// Deposit on high wall stake
				//cm->conveyer.move(true, false);
				pros::delay(2000);
				//cm->conveyer.move(false);
				//liftMech.actuate(false);
			}

			void linedAuton() {
				cm->dvt.PIDMove(30);
				cm->dvt.PIDTurn(-45);
				cm->dvt.moveDelay(500);
			}
			
			void calcCoefficientAuton()  {
				// 1 tile = 2 feet = 24 inches
				// 72 = 3 tiles = 3 feet
				// 96 = 4 tiles = 4 feet
				cm->dvt.PIDMove(-24);
				cm->doinker.actuate(true);
			}

			void testIMUAuton() {
				cm->dvt.moveVelocity(500, -500);
				pros::lcd::set_text(5, std::to_string(cm->dvt.getHeading()));
				pros::delay(10);
			}

			void calcTurnAuton() {
				cm->dvt.PIDTurn(-90);
			}

			void advancedAuton() {
				

				// TODO: add timer for PID functions to prevent infinite loops
				// Deposit preload on low wall stake
				pros::delay(2000);
				cm->dvt.PIDMove(8.8);
				pros::lcd::print(2, "Initial phase complete");
				pros::delay(500);

				// Move to mogo
				cm->dvt.PIDTurn(-90);
				cm->dvt.moveDelay(1600, false);
				cm->conveyer.move(true);
				pros::delay(1000);

				// stop it from hitting the wall
				cm->conveyer.move(false);
				cm->dvt.PIDMove(8);

				cm->dvt.PIDTurn(30);
				//cm->dvt.PIDMove(20);
				// uncommnet later

				// Turn halfway through going to mogo
				// fix to turn 180 degrees
				//return;
				// for some reason 90 degrees has become 180 degrees for some reason
				cm->dvt.PIDTurn(90);
				//dvt.PIDTurn(90);

				//return;

				pros::delay(200);
				//cm->dvt.PIDMove(-8);
				// uncommnet later

				// Collect mogo
				cm->mogoMech.actuate(true);
				pros::delay(500);

				//return;
				// Turn, move and collect rings
				cm->dvt.PIDTurn(-55);
				cm->conveyer.move(true);
				// uncommnet later
				//cm->dvt.PIDMove(25);
				pros::delay(500);
				cm->conveyer.move(false);

				// Prepare for opcontrol
				//cm->conveyer.move(false);

				// OPTIONAL: Turn to face the wall
				/*
				cm->dvt.PIDTurn(150);
				cm->dvt.PIDMove(70);
				*/
			}

			void aadiAuton() {
				cm->mogoMech.actuate(true);
                cm->dvt.moveSingleVelocity(1);
                cm->dvt.PIDMove(-29);
                pros::delay(500);
                cm->mogoMech.actuate(false);
                cm->conveyer.move(true);
                pros::delay(300);
                cm->dvt.PIDTurn(-57);
                cm->conveyer.move(true);
                cm->dvt.PIDMove(30);
			}

			void preRun() {
				cm->dvt.setBrakeModes(pros::E_MOTOR_BRAKE_HOLD);
			}

			void postRun() {
				cm->dvt.setBrakeModes(pros::E_MOTOR_BRAKE_COAST);
				cm->mogoMech.actuate(false);
			}
		protected:
		public:
			/// @brief Args for match auton object
			/// @param autonArgs Args for auton object
			struct MatchAutonArgs {
				AutonArgs autonArgs;
			};

			/// @brief Creates match auton object
			/// @param args Args for match auton object (check args struct for more info)
			MatchAuton(MatchAutonArgs args) : 
				AbstractAuton(args.autonArgs) {};

			// TODO: Implement
			void run() override {
				// just comment out the auton function u dont want

				preRun();

				//defaultAuton();
				//calcCoefficientAuton();
				//calcTurnAuton();
				//testIMUAuton();
				//linedAuton();
				//aadiAuton();
				advancedAuton();

				postRun();
			}
	}; // class MatchAuton

	class SkillsAuton : public AbstractAuton {
		private:
			void sector1() {
				cm->mogoMech.actuate(false);	
				cm->dvt.PIDMove(-9);
				cm->mogoMech.actuate(true);
				cm->conveyer.move(true);

				cm->dvt.PIDTurn(-30);
				cm->dvt.moveDelay(300);
				cm->dvt.PIDMove(6);
				cm->dvt.PIDTurn(180);
				cm->dvt.PIDMove(22);

				cm->dvt.PIDTurn(90);
				cm->dvt.PIDMove(22);
				cm->dvt.PIDTurn(90);
				cm->dvt.PIDMove(47);
				cm->dvt.PIDTurn(90);

				cm->dvt.moveDelay(400, false);
				cm->mogoMech.actuate(false);
				cm->dvt.PIDMove(5);
				cm->dvt.PIDTurn(90);
			}

			void sector2() {

			}
		protected:
		public:
			/// @brief Args for skills auton object
			/// @param autonArgs Args for auton object
			struct SkillsAutonArgs {
				AutonArgs autonArgs;
			};

			/// @brief Creates skills auton object
			/// @param args Args for skills auton object (check args struct for more info)
			SkillsAuton(SkillsAutonArgs args) : 
				AbstractAuton(args.autonArgs) {};

			// TODO: Implement
			void run() override {
				sector1();
				sector2();
			}
	}; // class SkillsAuton

	/// @brief Chassis class for controlling auton/driver control
	class Chassis : public AbstractChassis {
		private:
		protected:
		public:
			/// @brief Args for chassis object
			/// @param cmUserArgs Args for component manager object
			struct ChassisArgs {
				ComponentManager::ComponentManagerUserArgs cmUserArgs;
			};

			ComponentManager cm;

			MatchAuton matchAutonManager;
			SkillsAuton skillsAutonManager;

			/// @brief Creates chassis object
			/// @param args Args for chassis object (check args struct for more info)
			Chassis(ChassisArgs args) : 
				AbstractChassis(),
				cm({this, args.cmUserArgs}),
				matchAutonManager({&cm}),
				skillsAutonManager({&cm}) {};

			/// @brief Runs the opcontrol functions for each component
			void opControl() override {
				cm.opControl();
			}

			/// @brief Auton function for the chassis
			// 1000 = 70cm
			void auton() override {
				matchAutonManager.run();
			}

			/// @brief Skills auton function for the chassis
			void skillsAuton() override {
				skillsAutonManager.run();
			}

			/// @brief Skills preparation for opcontrol on the chassis
			void skillsPrep() override {
				cm.conveyer.move(true);
				cm.mogoMech.actuate(true);
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

	std::int32_t prepareMoveVoltage(float raw) {
		// Round the number to the nearest integer
		raw = std::round(raw);

		std::int32_t voltage = static_cast<std::int32_t>(raw);
		voltage = std::clamp(voltage, MotorBounds::MOVE_MIN, MotorBounds::MOVE_MAX);

		return voltage;
	}

	/// @brief Assert that a number is between two values
	/// @param num Number to assert
	/// @param min Minimum value
	/// @param max Maximum value
	template <typename T>
	bool isNumBetween(T num, T min, T max) {
		assertArithmetic(num);

		if ((num > min) && (num < max)) {
			return true;
		} else {
			return false;
		}
	}

	/// @brief Normalise an angle to the range [-180, 180]
	/// @param angle Angle to normalise
	template <typename T>
	T normaliseAngle(T angle) {
		assertArithmetic(angle);

		if (angle > 180) {
			angle -= 360;
		} else if (angle < -180) {
			angle += 360;
		}

		return angle;
	}

	/// @brief Naively normalise an angle to the range [-180, 180] by simply clamping the value
	/// @param angle Angle to normalise
	template <typename T>
	T naiveNormaliseAngle(T angle) {
		assertArithmetic(angle);

		angle = std::clamp(angle, -180.0, 180.0);

		return angle;
	}

	/*/// @brief Get all the values of an enum class into a vector
	template <typename T>
	vector<T> getAllValues() {
		vector<T> values;
		constexpr int max = static_cast<int>(T::_MAX);
		values.reserve(max);

		for (int i = 0; i < max; i++) {
			values.push_back(static_cast<T>(i));
		}

		return values;
	}

	/// @brief Fill a map with default values for an enum class (see below function def for example use case)
	/// @param map Map to fill
	template <typename E, typename V>
	void fillMapWithEnum(map<E, V>& map) {
		vector<E> values = getAllValues<E>();
		E defaultValue = V();

		for (E value : values) {
			map[value] = defaultValue;
		}
	}*/
	// example use case
	//fillMapWithEnum<pros::controller_digital_e_t, bool>(map);
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
		{{LEFT_DRIVE_PORTS, RIGHT_DRIVE_PORTS, IMU_PORT}, 
		MOGO_MECH_PORT, DOINKER_PORT, CONVEYER_PORTS, LADY_BROWN_PORTS}});
	
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
	if (DO_MATCH_AUTON) {
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

	bool inComp = pros::competition::is_connected();

	// competition auton test safeguard
	if (MATCH_AUTON_TEST && !inComp) {
		autonomous();
	}

	if (DO_SKILLS_PREP) {
		currentChassis->skillsPrep();
	}

	if (DO_SKILLS_AUTON) {
		currentChassis->skillsAuton();
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



		pros::delay(MAINLOOP_DELAY_TIME_MS);
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

// anti quick make nothing comment thingy
// aa
