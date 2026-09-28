# Robot: essential points

## Power

- PC battery: approximately 8-10 hours during intensive use.
- Motor batteries: keep above 48 V to preserve battery life.
- Motors stop working at about 46 V; full charge is approximately 53 V.
- Runtime: more than 30-40 hours, depending on daily use.
- LiDAR: it consumes too much power from the PC battery. It should be
	powered from the motor battery.

## Startup order

During development, the reliable startup sequence was:

1. Disconnect the LiDAR.
2. Start the Micro-ROS microcontrollers.
3. Reconnect the LiDAR.

The LiDAR could prevent the microcontrollers from starting if it was powered
first.

## Mechanical maintenance

- Regularly check and tighten the two screws securing the keys in each
	steering-belt gear.
- Check and tighten the wheel-to-chassis fasteners, especially the large belt
	gear connection. There are approximately eight screws.

## Remote control

Keep the joystick within approximately 15-20 m of the robot.

## Known rear-left wheel issue

The rear-left wheel lost control during the project. The drivers, ESP32
controller board, and the 8N optocoupler appeared to operate normally. The
controller board was replaced with the one from the arm robot, but this did
not solve the issue. ROS 2 commands were valid, yet the wheel did not respond
reliably; at one point the drive motor worked, but the steering servo did not.

The 2N optocoupler on the small ESP32-to-drive-driver board was not tested.
The exact fault was not identified.

## Ackermann configuration

To use the fallback Ackermann mode:

- The rear drive-driver power supplies were disconnected to free the rear
	wheels.
- The rear wheels were left non-driven and non-steered.

To restore full Swerve mode:

- Reconnect the rear drive-driver power supplies.
- Reinstall the robot's microcontroller board on the robot. It is stored in a
	bag next to the arm.
- Put the board currently installed on the robot back on the arm robot.

The Ackermann mode was used to continue testing despite the unresolved
hardware failure. Full Swerve mode remains the intended configuration.
