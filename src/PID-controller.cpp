#include "main.h"

// PID constants
const double Kp = 0.1;   // Proportional constant
const double Ki = 0.01;  // Integral constant
const double Kd = 0.1;   // Derivative constant

// Motor and sensor initialization
pros::Motor motor(1);  // Assuming the motor is on port 1
pros::ADIAnalogIn rotationSensor(1);  // Assuming the rotation sensor is on ADI port 1

// PID variables
double setpoint = 0.0; // Desired angle
double previousError = 0.0;
double integral = 0.0;
double dt = 20.0; // Loop interval in milliseconds

double getRotationAngle() {
    // Replace with actual conversion based on your sensor's output
    return rotationSensor.get_value(); // Get the sensor value
}

double PIDController(double currentAngle) {
    double error = setpoint - currentAngle;
    integral += error * dt;
    double derivative = (error - previousError) / dt;

    double output = Kp * error + Ki * integral + Kd * derivative;

    previousError = error;

    return output;
}

void controlLoop() {
    while (true) {
        double currentAngle = getRotationAngle();
        double motorOutput = PIDController(currentAngle);

        motor.move_velocity(motorOutput); // Set motor speed based on PID output

        pros::delay(dt); // Wait for the next loop
    }
}

int main() {
    setpoint = 90.0; // Example: Set desired angle to 90 degrees
    controlLoop();
}