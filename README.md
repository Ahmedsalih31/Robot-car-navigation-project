Robot Assembly and Navigation Project
Overview

This project implements an Arduino-based autonomous robot capable of multiple navigation behaviours using sensor-driven reactive control. The robot integrates an ultrasonic distance sensor mounted on a servo, a 5-channel infrared sensor array, and PWM-controlled DC motors to navigate its environment autonomously.

The system supports several operational modes including obstacle avoidance, line following, wall following, and combined navigation strategies, demonstrating practical embedded systems design and autonomous robotics principles.

Navigation Modes

The robot operates using selectable control modes defined in the code:

Mode 0 – Obstacle Avoidance
Uses ultrasonic distance sensing to detect obstacles ahead and dynamically selects a clear direction by scanning left and right with a servo-mounted sensor.

Mode 1 – Line Following
Uses a 5-channel IR sensor array to follow a line using reactive control and differential motor speeds.

Mode 2 – Wall Following
Maintains a fixed distance from a wall using ultrasonic feedback and proportional speed adjustments.

Mode 3 – Combined Obstacle + Line Following
Prioritises obstacle avoidance while defaulting to line following when the path is clear.

Mode 4 – Bug Algorithm (Goal Seeking + Wall Following)
Implements a simplified bug algorithm that switches between goal-seeking and wall-following behaviour when obstacles are encountered.

Hardware Components

Arduino microcontroller

Ultrasonic distance sensor (Trig/Echo)

Servo motor for sensor scanning

5-channel infrared sensor array

DC motors with motor driver

Buzzer for obstacle alerts

Mobile robot chassis and power supply

Pin Configuration
Motor Driver

Right Motor Direction: Pins 11, 12

Left Motor Direction: Pins 7, 8

Right Motor PWM: Pin 6

Left Motor PWM: Pin 5

Sensors and Actuators

Ultrasonic Trigger: Pin 10

Ultrasonic Echo: Pin 2

Servo Motor: Pin 9

Buzzer: Pin 13

IR Sensor Array

IR1–IR5: A1–A5

Control Parameters

Key parameters used to tune robot behaviour:

NORMAL_SPEED – Default forward speed

TURN_SPEED – Standard turning speed

SHARP_TURN_SPEED – High-speed turning

BACK_SPEED – Reverse speed

Distance thresholds for obstacle detection and wall following

Servo sweep angles for environmental scanning

These values can be adjusted to suit different environments and surfaces.

Code Structure

Motor control functions
moveForward(), moveBackward(), turnLeft(), turnRight(), curveLeft(), curveRight(), stopMotors()

Sensor functions
measureDistance() – Ultrasonic distance measurement
scanEnvironment() – Servo-based left/right scanning

Navigation logic
obstacleAvoidance()
lineFollowing()
wallFollowing()
combinedMode()
bugAlgorithm()

Utility functions
buzz() – Audible obstacle alert
