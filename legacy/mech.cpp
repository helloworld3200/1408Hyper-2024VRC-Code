// Legacy pneumatic mechanism code
// Moved from main.cpp file because it isn't used anymore

#include "main.h"

// needs abstractmech to compile
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
