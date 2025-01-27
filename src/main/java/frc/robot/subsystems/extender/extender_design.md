

# Extender

The extender comprises the elevator and the end effector pivot.

The operator wants to simply command the end effector to a specified height and angle, so these mechanisms are moved together.

These mechanisms should be controlled with an angle position pid for the pivot and a height position pid for the elevator.


## Elevator
- Controlled by two NEO's with a 4:1 gear ratio each.
- The motors power a pulley which moves the elevator up and down.
- Feedback is gotten through an absolute encoder mounted on the pulley shaft.
- The elevator is zeroed with a Time of Flight Sensor mounted pointing up to detect the position of the elevator when the elevator approaches the low position.
- The elevator is given a height to go to, and it uses PID control (on-motor if possible) to arrive at the height.  The PID controller also compensates for the extra weight of the elevator from gravity, potentially using Feedforward to compenate.

## End Effector Pivot
- Controlled by one NEO with a 25:1 gear ratio.
- The motor powers the pivot and angles the end effector.
- Feedback is gotten through an absolute encoder mounted on the pivot shaft.  This also means the mechanism doesn't have to zero at the beginning of the match.
- The mechanism is given a angle to go to, and it uses PID control to arrive at the angle.