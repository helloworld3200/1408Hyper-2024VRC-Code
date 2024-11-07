#ifndef _HYPER_AUTON_HPP_
#define _HYPER_AUTON_HPP_

#include "main.h"

/// @brief Our namespace for all custom classes and functions
namespace hyper {
    /// @brief Main auton class
	class MatchAuton {
		private:
			Chassis* chassis;

			void driveSector1() {
				chassis->dvt.turnTo(45);
				chassis->intake.move(true);

			};
		protected:
		public:
			int speed = 100;

			/// @brief Creates auton object
			/// @param chassis Pointer to chassis object
			MatchAuton(Chassis* chassis) : 
				chassis(chassis) {};

			void go() {
				driveSector1();
			};
	}; // class Auton
}

#endif // _HYPER_AUTON_HPP_