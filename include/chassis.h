// Chassis code
#include "main.h"

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
	public:
		/// @brief Args for ChassisComponent object
		/// @param chassis AbstractChassis derived object to be used for the component
		struct ChassisComponentArgs {
			AbstractChassis* chassis;
		};

		/// @brief Creates ChassisComponent object
		/// @param args Args ChassisComponent object (check args struct for more info)
		ChassisComponent(ChassisComponentArgs args) : chassis(args.chassis) {};
		virtual ~ChassisComponent() = default;
};
