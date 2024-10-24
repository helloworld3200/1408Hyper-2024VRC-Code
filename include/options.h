// All the options for the robot

#ifndef _HYPER_OPTIONS_H_
#define _HYPER_OPTIONS_H_

// Variables (u can change these!!)
#define DELAY_TIME_MS 20
// turn on for auton to be run at the start of opcontrol
#define AUTON_TEST false

#define CURRENT_OPCONTROL mainControl

// Digital sensor port for pneumatics mogo mech
#define MOGO_MECH_PORT 'B'

// Digital sensor port for pneumatics conveyor lift
#define LIFT_MECH_PORT 'A'

// Motor ports for the conveyer
#define CONVEYER_PORTS {11, -1}

// Turn on/off auton and opcontrol
#define DO_AUTON true
#define DO_OP_CONTROL true

// Ports for the drivetrain motors
#define LEFT_DRIVE_PORTS {10, 9, 8}
#define RIGHT_DRIVE_PORTS {20, 19, 18}

// Chassis class to use (default is initDefaultChassis)
#define INIT_CHASSIS initDefaultChassis

#endif // _HYPER_OPTIONS_H_
