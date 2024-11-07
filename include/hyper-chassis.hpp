#ifndef _HYPER_CHASSIS_HPP_
#define _HYPER_CHASSIS_HPP_

#include "main.h"
#include "hyper-auton.hpp"

/// @brief Our namespace for all custom classes and functions
namespace hyper {
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

			MatchAuton autonController;

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
}

#endif // _HYPER_CHASSIS_HPP_