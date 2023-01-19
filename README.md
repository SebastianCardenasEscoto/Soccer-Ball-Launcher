# Soccer-Ball-Launcher
Soccer Ball Launcher with Computer Controlled pitch and yaw axis

## Hardware Setup:

<img src="https://i.imgur.com/kWsSIrV.png">

https://forum.arduino.cc/t/xy-plotter-wiring-help-needed-cnc-shield-2-stepper-motors-1-solenoid/654223

The current Arduino code expects the yaw stepper to be connected to the X Stepper and the pitch stepper to be connected to the Y stepper.
We found that the power supply current limit must be set to be much higher than the current these motors draw, we suspect this is due to startup torque. We had success with about 3A.

As of now the brushed motors for the launch wheels haven’t been tested. We had planned to control the speed by supplying a fixed voltage and adjusting our python code to account for the launch velocity (that we would find experimentally). We had some encoders, but didn’t have time to implement them previously due to the tight deadline. We can implement them on request.

## Software Setup:

Flash movePitchAndYaw.ino onto the Arduino. This code is expecting serial commands from getTargetCoordinates.py. The Arduino needs to be reset in order to send new coordinates.

The Python script is set up to detect an AR tag in the camera frame and calculate the pitch and yaw angles needed to hit it. The script then sends the angles over serial to the Arduino. We can reconfigure the code to take user input for angles and send those instead of using the ar tag, email us if this is desired.

