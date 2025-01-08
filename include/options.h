// All the options for the robot

#ifndef _HYPER_OPTIONS_H_
#define _HYPER_OPTIONS_H_

// Variables (u can change these!!)
#define MAINLOOP_DELAY_TIME_MS 20
// Turn on for match auton to be run at the start of opcontrol
#define MATCH_AUTON_TEST false

#define CURRENT_OPCONTROL mainControl

// Digital sensor port for pneumatics mogo mech
#define MOGO_MECH_PORT 'A'

// Digital sensor port for pneumatics conveyor lift
#define LIFT_MECH_PORT 'B'

// Digital sensor port for pneumatics doinker
#define DOINKER_PORT 'B'

// Ultrasound (1st is ping port, 2nd is echo port)
#define BACK_ULTRA_PORTS {'A', 'B'}

// Motor ports for the conveyer (real is 10 but use 11 to just turn it off for debugging)
#define CONVEYER_PORTS {4, -10}

// Motor ports for the intake motor group
#define INTAKE_PORTS {11, -12}

// Motor ports for the lady brown mech
#define LADY_BROWN_PORTS {1}

// Port for the color sensor
#define COLOR_SENSOR_PORT 5

// Ports for telemetry
// IMU
#define IMU_PORT 12

// Turn on/off auton and opcontrol
#define DO_MATCH_AUTON true
#define DO_SKILLS_PREP true
#define DO_SKILLS_AUTON false
#define DO_OP_CONTROL true

// Ports for the drivetrain motors
#define LEFT_DRIVE_PORTS {6, 7, 18}
#define RIGHT_DRIVE_PORTS {-13, -14, -15}

// Chassis class to use (default is initDefaultChassis)
#define INIT_CHASSIS initDefaultChassis

#endif // _HYPER_OPTIONS_H_
