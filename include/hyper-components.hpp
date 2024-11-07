#ifndef _HYPER_COMPONENTS_HPP_
#define _HYPER_COMPONENTS_HPP_

#include "main.h"

/// @brief Our namespace for all custom classes and functions
namespace hyper {
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

			pros::IMU imu;

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
				std::int8_t imuPort;
			};

			/// @brief Args for drivetrain object
			/// @param abstractComponentArgs Args for AbstractComponent object
			struct DrivetrainArgs {
				AbstractComponentArgs abstractComponentArgs;
				DrivetrainPorts ports;
			};

			DriveControlSpeed driveControlSpeed = {};

			std::int32_t maxRelativeVelocity = 1024;
			std::int8_t maxRelativeError = 5;

			std::int16_t maxTurnVelocity = 127;
			float minTurnThreshold = 1;

			uint32_t moveDelayMs = 2;

			Drivetrain(DrivetrainArgs args) : 
				AbstractComponent(args.abstractComponentArgs),
				left_mg(args.ports.left),
				right_mg(args.ports.right),
				imu(args.ports.imuPort) {
					setDriveControlMode();
					calibrateIMU();
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
				
				std::int32_t left_voltage = prepareMoveVoltage(dir - turn);                      // Sets left motor voltage
				std::int32_t right_voltage = prepareMoveVoltage(dir + turn);                     // Sets right motor voltage

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
			/// @param pos Position to move to
			void moveRelPos(double pos) {
				tareMotors();

				left_mg.move_relative(pos, maxRelativeVelocity);
				right_mg.move_relative(pos, maxRelativeVelocity);

				double lowerError = pos - maxRelativeError;
				double upperError = pos + maxRelativeError;

				while ((
					!isNumBetween(left_mg.get_position(), lowerError, upperError)
				) && (
					!isNumBetween(right_mg.get_position(), lowerError, upperError)
				)) {
					pros::delay(moveDelayMs);
				}
			}

			/// @brief Turn to a specific angle
			/// @param angle Angle to turn to
			void turnTo(double angle) {
				double currentHeading = imu.get_heading();
				double angleDifference = normaliseAngle(angle - currentHeading);

				std::int16_t turnDirection = (angleDifference > 0) ? maxTurnVelocity : -maxTurnVelocity;
				
				left_mg.move_velocity(turnDirection);
				right_mg.move_velocity(-turnDirection);

				while (std::abs(angleDifference) > minTurnThreshold) {
					currentHeading = imu.get_heading();
					angleDifference = normaliseAngle(angle - currentHeading);

					if (angleDifference > 180) {
						angleDifference -= 360;
					} else if (angleDifference < -180) {
						angleDifference += 360;
					}

					pros::delay(moveDelayMs);
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
			/// @brief Args for pointers required for conveyer object
			/// @param mogoMech Pointer to mogo mech object
			/// @param liftMech Pointer to lift mech object
			struct ReqPointers {
				MogoMech* mogoMech;
				LiftMech* liftMech;
			};

		private:
			ReqPointers reqPointers;

			BiToggle toggle;
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

			Conveyer(ConveyerArgs args) :
				AbstractMG(args.abstractMGArgs), 
				reqPointers(args.reqPointers),
				toggle({this, {
					pros::E_CONTROLLER_DIGITAL_L2,
					pros::E_CONTROLLER_DIGITAL_L1
				}}) {};

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
					pros::E_CONTROLLER_DIGITAL_R1,
					pros::E_CONTROLLER_DIGITAL_R2
				}}) {}

			bool canMove(bool on) override {
				return on;
			}

			void opControl() override {
				toggle.opControl();
			}
	}; // class Intake
} // namespace hyper

#endif // _HYPER_COMPONENTS_HPP_