#ifndef _HYPER_BASE_HPP_
#define _HYPER_BASE_HPP_

#include "main.h"

/// @brief Our namespace for all custom classes and functions
namespace hyper {
	// Function declarations
	template <typename T>
	string vectorToString(vector<T>& vec, string delimiter = ", ");

	template <typename T>
	T clamp(T val, T min, T max);

	std::int32_t prepareMoveVoltage(float raw);

	template <typename T>
	bool isNumBetween(T num, T min, T max);

	template <typename T>
	T normaliseAngle(T angle);

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
		private:
			enum class State {
				OFF,
				FWD,
				BACK
			};

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
				} else {
					moveState(State::FWD);
				}
			}

			void handleBackBtn() {
				if (state == State::BACK) {
					moveState(State::OFF);
				} else {
					moveState(State::BACK);
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

				if (fwdPressed && backPressed) {
					// Don't do anything if both are pressed
					// TODO: test whether returning actually works
					//return;
				}

				if (fwdPressed) {
					handleFwdBtn();
					isNewPress = false;
				} else if (backPressed) {
					handleBackBtn();
					isNewPress = false;
				} else {
					isNewPress = true;
				}
			}
	}; // class BiToggle

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

	std::int32_t prepareMoveVoltage(float raw) {
		// Round the number to the nearest integer
		raw = std::round(raw);

		std::int32_t voltage = static_cast<std::int32_t>(raw);
		voltage = clamp(voltage, MotorBounds::MOVE_MIN, MotorBounds::MOVE_MAX);

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
} // namespace hyper

#endif // _HYPER_BASE_HPP_