# BARD: Autonomous Desk Cleaning Robot

ECE4180 Final Project

### Developer
Jihwan Oh <br>
JongWook Lee

## Overview

While robot vacuums in the market today clean our floors, they are not built for desks and often fall off edges. To close this gap, we developed BARD, an autonomous desk cleaning robot built on the Arduino Mega 2560. BARD can autonomously navigate around obstacles and cliffs, determine optimal movement paths by using  the two servo-mounted ultrasonic sensors rotating on the top. The attached LCD displays status updates, and the weight sensing load cell tells the user when the bin is full and needs emptying.

## Features

### Cliff & Obstacle Detection:
Prevents falling off edges by measuring front and back distances.
By nodding the head, it detects the cliff and obstacle.

### Autonomous Movement:
Decides optimal direction based on distance mapping.

### Load Monitoring:
Monitors collected trash weight using a load cell.

### User Interface:
Displays real-time status messages on a uLCD screen.
Also, show the bin condition (full or clean)

### Vacuum Control:
Integrates a vacuum motor for active dust collection.

## Hardware Components

Arduino Mega 2560 : Serves as the MCU of this robot. Provides robust processing power and abundant I/O pins.

2x Ultrasonic Sensors : Sensors measure distance to obstacles and cliffs. Collected real- time readings feed into the navigation algorithm.

2x Servo Motors (for base and head movement) : Attached to each other, delivers two-axis rotation. This allows the two ultrasonic sensors to scan the surroundings.

DC Motor Driver (with standby control) & 2x DC Motor (Left and Right wheels) : Drives the robot’s wheels for movement.

Vacuum Motor : Creates suction via a propeller housed in a 3d printed funnel.

HX711 Load Cell Amplifier Module :Amplifies the load cell’s signal for the Arduino to read it accurately.

Load Cell (Weight Sensor) : Senses the weight of the collected debris in the bin. Allows to check if the bin is full.

4D Systems uLCD : Displays the robot’s state with three different modes: Cleaning(Smile), Edge Detected, and Bin Full.

2x Toggle Switch (Display Mode change and turn on/off vacuum) One used to turn the vacuum power on/off. The other used to change the LCD display mode.


## Pin Configuration and Circuit Diagram

| Component                     | Pin    |
|------------------------------ |----------------|
| Base Servo                    | 7              |
| Rotation Servo                | 8              |
| Sonar 1 (Trig/Echo)           | 13 / 12        |
| Sonar 2 (Trig/Echo)           | 11 / 10        |
| Left Motor (PWM/Dir1/Dir2)    | 6 / 22 / 23    |
| Right Motor (PWM/Dir1/Dir2)   | 9 / 24 / 25    |
| Motor Driver Standby          | 29             |
| HX711 (Data/Clock)            | 3 / 5          |
| uLCD (Serial + Reset)         | Serial1 / 4    |
| Vacuum Polling Pin            | 2              |
<p align="center">
  <img width="50%" alt="image" src="https://github.com/user-attachments/assets/7393bf4f-599a-47e5-9886-181d7ed11bcb" />
</p>


## Main Loop (loop)

1. around dist measure
2. get medium angle + rotate (algorithm)
3. nodding + detect obstacle and cliff
4. move forward, backward or rotate (3 condition)

If the vacuum motor signals a full bin, measure the bin weight and display the result.

## Libraries Used

1. Goldelox-Serial-Arduino-Library: https://github.com/4dsystems/Goldelox-Serial-Arduino-Library

The library was used to easily display the desired message and graphics.


2. HX711 Library: https://github.com/bogde/HX711

The library was used to calibrate the scaling factor, initialize the sensor, and retrieve the readings. 

3. Servo Library: https://docs.arduino.cc/libraries/servo/

The library was used to control the servo motors easily.

## Problems Encountered During this Project
There were multiple issues that we faced when using the peripherals. For example, when using two ultrasonic sensors simultaneously, interference between the emitted beams occurred, leading to incorrect distance measurement. To solve this, we added delays into each step of sensing. Also, the emitted beam from the sensor needs to be perpendicular to the surface; failing this resulted in measurements with a high variance. We increased the length of sonar sensor’s neck to enable a full 180-degree sweep. When generating the suction using the dc motor, we initially used a rectangular-shape tube for the container. However, the power was not enough to collect the trash, so we later changed into a tapered funnel.  Lastly, using cardboard as the plate on the load cell gave inconsistent values; therefore, we 3D-printed a rigid plate, which produced consistent readings.

## How is your project similar and different and similar from other real world embedded systems?
Our project shares some core ideas from commercial robotic vacuums in terms of autonomous navigation, a vacuum motor, and microcontroller driven motor control. However, BARD is different from them in two ways. First, instead of falling off the cliff, by using the data collected from the ultrasonic sensors that rotate through 180-degree sweep, it can prevent from tumbling off the cliff. Second, by integrating the load cell and the dedicated amplifier, BARD can measure the weight of the collected trash and notify the user to clean the bin when necessary. 
## Potential Improvements. 
- PID control for more precise movement.
- Battery monitoring system.
- Improved weight calibration and adaptive cleaning strategies.
- Wireless status reporting (Bluetooth / Wi-Fi).
- Motor powered by solar panels

# Demo
<p align="center">
  <img src="https://i.imgur.com/z9OD3dd.gif" width="15%" />
  <img src="https://imgur.com/O80AWOy.gif" width="15%" />
  <img src="https://imgur.com/my7WCZA.gif" width="15%" />
  <img src="https://imgur.com/x6ajDim.gif" width="15%" />
</p><p align="center">
  <img src="https://imgur.com/WL8vLB8.gif" width="15%" />
  <img src="https://imgur.com/bov9D1O.gif" width="15%" />
  <img src="https://imgur.com/MvnGggd.gif" width="15%" />
</p>


