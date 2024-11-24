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

// TODO: Create drivetrain utility functions e.g. forward, turn degrees, etc.
// (with PID!!!! SPECIFICALLY turning and lateral)

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
			virtual void skills() = 0;
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

			pros::Controller& getMaster() {
				return *master;
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
				int fwd = 10000;
				int back = -10000;
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

					DriveControlSpeed(float turnSpeed = 2, float forwardBackSpeed = 2, float arcSpeed = 0.6) :
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

			/// @brief Struct for PID options (self-explanatory)
			struct PIDOptions {
				double kP;
				double kI;
				double kD;
				double errorThreshold;
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

			double inchesPerTick = 0.025525;

			uint32_t moveDelayMs = 2;

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
				lateral *= 1;

				// Clamp the range to above 0 only to remove back movement
				if (preventBackMove && (lateral < 0)) {
					lateral = 0;
				}

				lateral *= driveControlSpeed.getForwardBackSpeed();
			}

			// Calculate the movement of the robot when turning and moving laterally at the same time
			void calculateArcMovement(TurnCoefficients& turnCoeffs, float lateral, float turn, float maxLateralTolerance = 1) {
				// 0-1 range of percentage of lateral movement against max possible lateral movement
				float lateralCompensation = lateral / driveControlSpeed.getMaxLateral();
				// Decrease the turn speed when moving laterally
				float turnDecrease = turn * driveControlSpeed.arcSpeed * lateralCompensation;

				if (turn > 0) { // Turning to right so we decrease the left MG
					turnCoeffs.left -= turnDecrease;
				} else { // Turning to left so we decrease the right MG
					turnCoeffs.right -= turnDecrease;
				}
			}

			TurnCoefficients calculateArcadeTurns(float turn, float lateral) {
				turn *= -1;
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

				left_mg.move(left_voltage);
				right_mg.move(right_voltage);
			}

			/// @brief Fallback control that DriveControlMode switch statement defaults to.
			void fallbackControl() {
				arcadeControl();
			}

			/// @brief Calibrates the IMU
			void calibrateIMU() {
				imu.reset();
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
					moveSingleVelocity(defaultMoveVelocity);
				} else {
					moveSingleVelocity(-defaultMoveVelocity);
				}

				pros::delay(delayMs);
				moveStop();
			}

			// TODO: Generic PID function that we can apply to PIDTurn and PIDMove
			// maybe make a class for this? if it gets too complicated
			// but that would also require refactoring Drivetrain to have an AbstractDrivetrain
			// parent to avoid cyclic dependencies

			// WARNING: do NOT use relativeMovementCoefficient for PID functions
			// as this does not account for acceleration/deceleration
			// it's only for simple movement (phased out by PID & PIDOptions struct)

			/// @brief PID Turn to specific angle
			/// @param angle Angle to move to (PASS IN THE RANGE OF -180 TO 180 for left and right)
			// TODO: Tuning required
			void PIDTurn(double angle, PIDOptions options = {
				0.1, 0.1, 0.1, 1
			}) {
				imu.tare();
				angle = naiveNormaliseAngle(angle);

				// IMU already tared so we don't need to get the current heading
				float error = angle;
				float lastError = 0;
				float derivative = 0;
				float integral = 0;
				float out = 0;

				// with turning you just wanna move the other MG at negative of the MG of the direction
				// which u wanna turn to

				while (true) {
					error = naiveNormaliseAngle(angle - imu.get_heading());

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
				}

				moveStop();
			}

			// think about arc motion, odometry, etc.
			// the key thing is PID.
			// TUNING REQUIRED!!!

			/// @brief PID Move to specific position
			/// @param pos Position to move to in inches (use negative for backward)
			// TODO: Tuning required
			void PIDMove(double pos, PIDOptions options = {
				0.1, 0.1, 0.1, 5
			}) {
				// TODO: Consider adding odometry wheels as the current motor encoders
				// can be unreliable for long distances
				tareMotors();

				pos /= inchesPerTick;

				float error = pos;
				float lastError = 0;
				float derivative = 0;
				float integral = 0;
				float out = 0;

				// with moving you just wanna move both MGs at the same speed

				while (true) {
					// get avg error
					error = pos - (left_mg.get_position() + right_mg.get_position()) / 2;

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
		public:
			/// @brief Args for pointers required for conveyer object
			/// @param mogoMech Pointer to mogo mech object
			/// @param liftMech Pointer to lift mech object
			struct ReqPointers {
				MogoMech* mogoMech;
				LiftMech* liftMech;
			};

		private:
			ReqPointers reqPointers;
		protected:
		public:
			BiToggle toggle;

			/// @brief Args for conveyer object
			/// @param abstractMGArgs Args for AbstractMG object
			/// @param conveyerPorts Vector of ports for conveyer motors
			/// @param mogoMech Pointer to mogo mech object
			struct ConveyerArgs {
				AbstractMGArgs abstractMGArgs;
				ReqPointers reqPointers;
			};

			Conveyer(ConveyerArgs args) :
				AbstractMG(args.abstractMGArgs), 
				reqPointers(args.reqPointers),
				toggle({this, {
					pros::E_CONTROLLER_DIGITAL_R2,
					pros::E_CONTROLLER_DIGITAL_R1
				}}) {
					speeds = {250, -250};
				};

			bool canMove(bool on) override {
				bool mogoMechMoving = reqPointers.mogoMech->getEngaged();
				bool liftMechMoving = reqPointers.liftMech->getEngaged();

				bool moveConveyer = (mogoMechMoving && on) || (liftMechMoving && on);

				return moveConveyer;
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

			struct Speeds {
				int fwd = 1000;
				int back = -1000;
			};

			Speeds speeds = {};

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

	/// @brief Class for controlling the stopper based on the color sensor
	class ColorStopper : public AbstractComponent {
		private:
			Conveyer* conveyer;
			LiftMech* liftMech;

			pros::Optical colorSensor;

			bool doStop = false;
		protected:
		public:
			/// @brief Args for color stopper object
			/// @param abstractComponentArgs Args for AbstractComponent object
			struct ColorStopperArgs {
				AbstractComponentArgs abstractComponentArgs;
				std::int8_t colorSensorPort;
				Conveyer* conveyer;
				LiftMech* liftMech;
			};

			struct ColorThresholds {
				float lower;
				float upper;
			};

			vector<ColorThresholds> blue = {{0, 30}, {330, 360}};
			vector<ColorThresholds> red = {{180, 240}};

			pros::controller_digital_e_t btn = pros::E_CONTROLLER_DIGITAL_Y;

			/// @brief Creates color stopper object
			/// @param args Args for color stopper object (check args struct for more info)
			ColorStopper(ColorStopperArgs args) : 
				AbstractComponent(args.abstractComponentArgs),
				colorSensor(args.colorSensorPort),
				conveyer(args.conveyer),
				liftMech(args.liftMech) {};

			bool isColor(vector<ColorThresholds>& thresholds) {
				for (ColorThresholds threshold : thresholds) {
					float reading = colorSensor.get_hue();
					if (isNumBetween(reading, threshold.lower, threshold.upper)) {
						return true;
					}
				}

				return false;
			}

			void checkStopColor(vector<ColorThresholds>& thresholds) {
				if (isColor(thresholds)) {
					conveyer->move(false);
					liftMech->actuate(true);
					conveyer->toggle.setState(BiToggle::State::OFF);
					doStop = false;
				}
			}

			/// @brief Runs every loop to check if the button has been pressed
			void opControl() override {
				if (master->get_digital(btn)) {
					doStop = true;
				}

				if (doStop) {
					checkStopColor(blue);
					checkStopColor(red);
				}
			}
	};

	/// @brief Class for stopping based on the ultrasonic sensor
	class UltraStopper : public AbstractComponent {
		private:
		protected:
		public:
			/// @brief Args for ultra stopper object
			/// @param abstractComponentArgs Args for AbstractComponent object
			/// @param backUltraPorts Vector of ports for back ultra sensor
			struct UltraStopperArgs {
				AbstractComponentArgs abstractComponentArgs;
				vector<char> backUltraPorts;
				Drivetrain* dvt;
			};

			Drivetrain* dvt;

			pros::adi::Ultrasonic ultra;

			float threshold = 10;

			/// @brief Creates ultra stopper object
			/// @param args Args for ultra stopper object (check args struct for more info)
			UltraStopper(UltraStopperArgs args) : 
				AbstractComponent(args.abstractComponentArgs),
				ultra(args.backUltraPorts[0], args.backUltraPorts[1]),
				dvt(args.dvt) {};

			void opControl() override {
				float distance = ultra.get_value();

				if (distance <= threshold) {
					dvt->preventBackMove = true;
				} else {
					dvt->preventBackMove = false;
				}
			}
	};

	/// @brief Chassis class for controlling auton/driver control
	class Chassis : public AbstractChassis {
		private:
			void defaultAuton() {
				// destruction 100

				/*//dvt.moveRelPos(300);
				//
				dvt.turnDelay(true, 600);
				dvt.moveRelPos(100);
				dvt.turnDelay(false, 400);
				//dvt.moveRelPos(150);
				dvt.turnDelay(true, 300);*/

				/*intake.move(true, false);
				conveyer.move(true);*/

				// Because auton is only 15 secs no need to divide into sectors
				// Move and collect first rings/discombobulate first
				//intake.move(true);
				dvt.turnDelay(true, 600);
				//pros::delay(MAINLOOP_DELAY_TIME_MS);
				dvt.moveRelPos(50);

				// Get the far ring and turn back onto main path
				// (no longer necessarily needed because we start with 1 ring already in the robot)
				dvt.turnDelay(true, 330);
				dvt.moveRelPos(105);
				//pros::delay(MAINLOOP_DELAY_TIME_MS);
				//dvt.turnDelay(false, 1.5);

				// Get other stack knocked over
				// optional: increase speed to intake if we have no harvester
				//dvt.moveRelPos(130);
				//dvt.moveDelay(600, false);
				dvt.turnDelay(false, 450);
				//pros::delay(MAINLOOP_DELAY_TIME_MS);
				dvt.moveRelPos(160);
				
				// Turn into high wall stake & deposit
				dvt.turnDelay(false, 870);
				dvt.moveDelay(800, false);
				//intake.move(false);
				//liftMech.actuate(true);
				//pros::delay(MAINLOOP_DELAY_TIME_MS);

				// Deposit on high wall stake
				//conveyer.move(true, false);
				pros::delay(2000);
				//conveyer.move(false);
				//liftMech.actuate(false);
			}

			void linedAuton() {
				dvt.moveRelPos(10);
			}
			
			void calcCoefficientAuton()  {
				dvt.moveRelPos(100);
			}

			void skillsSector1() {
				mogoMech.actuate(false);
				dvt.moveDelay(300, false);
				mogoMech.actuate(true);
				dvt.turnDelay(false, 600);
				intake.move(true);
				conveyer.move(true);

				dvt.moveRelPos(90);
				dvt.turnDelay(false, 400);
				dvt.moveDelay(300, false);

				mogoMech.actuate(false);
			}

			void skillsSector2() {

			}
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
				std::int8_t colorSensorPort;
				vector<char> backUltraPorts;
			};

			Drivetrain dvt;

			MogoMech mogoMech;
			LiftMech liftMech;

			Conveyer conveyer;
			Intake intake;

			/*ColorStopper colstop;
			UltraStopper ultraStopper;*/

			/// @brief Creates chassis object
			/// @param args Args for chassis object (check args struct for more info)
			Chassis(ChassisArgs args) :  
				dvt({this, args.dvtPorts}),
				mogoMech({this, args.mogoMechPort}), 
				liftMech({this, args.liftMechPort}), 
				conveyer({{this, args.conveyerPorts}, {&mogoMech, &liftMech}}), 
				intake({this, args.intakePorts})/*,
				colstop({this, args.colorSensorPort, &conveyer, &liftMech})
				ultraStopper({this, args.backUltraPorts, &dvt})*/ {};

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

				// Misc (eg color sensor)
				/*colstop.opControl();
				ultraStopper.opControl();*/
			}

			/// @brief Auton function for the chassis
			// 1000 = 70cm
			void auton() override {
				//defaultAuton();
				//calcCoefficientAuton();
				linedAuton();
			}

			void skills() override {
				skillsSector1();
				skillsSector2();
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

		angle = std::clamp(angle, -180, 180);

		return angle;
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
		{LEFT_DRIVE_PORTS, RIGHT_DRIVE_PORTS, IMU_PORT}, 
	MOGO_MECH_PORT, LIFT_MECH_PORT, CONVEYER_PORTS, INTAKE_PORTS, 
	COLOR_SENSOR_PORT, BACK_ULTRA_PORTS});
	
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

	if (MATCH_AUTON_TEST) {
		autonomous();
	}

	if (DO_SKILLS_AUTON) {
		currentChassis->skills();
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
