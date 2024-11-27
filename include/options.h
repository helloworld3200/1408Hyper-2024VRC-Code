// All the options for the robot

#ifndef _HYPER_OPTIONS_H_
#define _HYPER_OPTIONS_H_

// Variables (u can change these!!)
#define MAINLOOP_DELAY_TIME_MS 20
// Turn on for match auton to be run at the start of opcontrol
#define MATCH_AUTON_TEST true

#define CURRENT_OPCONTROL mainControl

// Digital sensor port for pneumatics mogo mech
#define MOGO_MECH_PORT 'B'

// Digital sensor port for pneumatics conveyor lift
#define LIFT_MECH_PORT 'A'

// Motor ports for the conveyer
#define CONVEYER_PORTS {-16, 10}

// Motor ports for the intake motor group
#define INTAKE_PORTS {-2, 3}

// Port for the color sensor
#define COLOR_SENSOR_PORT 5

// Ports for telemetry
// IMU
#define IMU_PORT 15
// Ultrasound (1st is ping port, 2nd is echo port)
#define BACK_ULTRA_PORTS {'A', 'B'}

// Turn on/off auton and opcontrol
#define DO_MATCH_AUTON true
#define DO_SKILLS_AUTON false
#define DO_OP_CONTROL true

// Ports for the drivetrain motors
#define LEFT_DRIVE_PORTS {-6, -7,- 8}
#define RIGHT_DRIVE_PORTS {17, 13, 16}

// Chassis class to use (default is initDefaultChassis)
#define INIT_CHASSIS initDefaultChassis

#endif // _HYPER_OPTIONS_H_
