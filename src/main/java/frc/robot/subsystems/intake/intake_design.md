# Intake

The intake mechanism is the funnel that guides coral into the end effector and the feeder wheel that pushes the coral into the end effector.

The intake takes coral from the coral station and feeds it into the end effector.


## Feeder Wheel
- The wheel is run by a NEO motor.
- Velocity control.
- Told to spin at a speed setpoint, then runs at that setpoint

## Coral Detector
- A beam break sensor mounted at the same location as the the feeder wheel.
- You should wait until the coral has been detected, the feeder wheel pushes it into the end effector, and the coral detector no longer detects a coral.
