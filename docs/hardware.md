## Hardware Description for the Break-a-Bot
Last updates: May 21st, 2026, by G. Gorospe

About: The break-a-bot (BB) is a triangular mobile robot with 3 omni-wheels located at the three corners of the triangle. Each wheel is driven by a brushed DC motor controlled by a RoboteQ SDC2130. This configuration produces an omni-directional robot and is sometimes called a “Kiwi drive.” Although each SDC2130 has two channels and can control two motors, in our configuration both channels are connected to the motor through the use of 6 relays that are used to dynamically change which SDC2130 channel is physically connected to the motor. This configuration allows for physical SPST switch to inject an abrupt open-circuit fault in the controller-to-motor circuit. There are 6 total SPST switches used for high-side open-circuit fault injection on channel 1 and 2 of each of the three SDC2130 controllers.

[Break-a-bot Hardware Block Diagram](/diagrams/Break_a_bot_Hardware_BlockDiagram.png)

Robot Parameters:
Omni-wheel diameter: 48 mm
Triangle-Side distance: L: 14”

Motor Information:
Brand: Pittman LO-COG
Model/Series: F5019
Specifications: 400 CPR “cycles per revolution” RATIO S 20-30 “gear ratio”
Characteristics: 
•	designed for low inertia, 
•	fast response, and 
•	smooth operation.

Motor Controller Information
RoboteQ SDC2130 x 3 (one controller for each motor)
Sensing: encoder, battery voltage, battery temperature

Battery Information: 
A single 4S 14.8 VDC LiPo battery powering all three motor controllers and the compute element. 

Compute Information:
Raspberry Pi 5 w/ ROS2 (Humble)
OS: Ubuntu 22.04
The raspberry pi is powered through it’s USB-C port via a “Power supply Expansion board for raspberry pi” YAHBOOM SKU: RM-YAHB-06K
It supports 6~24V voltage input and 5V/5A voltage output.

Relay board for abrupt fault injection:
SunFounder 5V 8 Channel Relay Shield Module for Raspberry Pi /Arduino
•	5V 8-Channel Relay interface board and each one needs 15-20mA Driver Current
•	Equipped with high-current relay, AC250V 10A ; DC30V 10A
•	Standard interface that can be controlled directly by microcontroller (Arduino, 8051, AVR, PIC, DSP, ARM, ARM, MSP430, TTL logic)

The 8 relay board is controlled by the Raspberry pi and can be commanded to open/close the circuits between the motor controllers and the DC motors.

Additional Sensor Information:
Adafruit 9-DOF Absolute Orientation IMU Fusion Breakout - BNO055
The BNO055 can output the following sensor data:
●	Absolute Orientation (Euler Vector, 100Hz) Three axis orientation data based on a 360° sphere
●	Absolute Orientation (Quatenrion, 100Hz) Four point quaternion output for more accurate data manipulation
●	Angular Velocity Vector (100Hz) Three axis of 'rotation speed' in rad/s
●	Acceleration Vector (100Hz) Three axis of acceleration (gravity + linear motion) in m/s^2
●	Magnetic Field Strength Vector (20Hz) Three axis of magnetic field sensing in micro Tesla (uT)
●	Linear Acceleration Vector (100Hz) Three axis of linear acceleration data (acceleration minus gravity) in m/s^2
●	Gravity Vector (100Hz) Three axis of gravitational acceleration (minus any movement) in m/s^2
●	Temperature (1Hz) Ambient temperature in degrees celsius


Camera:
Raspberry Pi Global Shutter Camera with 6 mm Wide Angle Lens


