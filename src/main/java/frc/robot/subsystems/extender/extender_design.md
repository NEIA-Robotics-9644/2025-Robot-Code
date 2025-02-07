# Extender
- The extender comprises the elevator and the end effector pivot.
- The operator wants to simply command the end effector to a specified height and angle, so these mechanisms are moved together.
- These mechanisms should be controlled with an angle position pid for the pivot and a height position pid for the elevator.

## Elevator
- Controlled by two NEO's with a 4:1 gear ratio each.
- The motors power a pulley which moves the elevator up and down.
- Feedback is gotten through an REV throughbore absolute encoder mounted on the pulley shaft.  The encoder is run in absolute mode
- The encoder goes from 0-1 each rotation.  Because the shaft rotates multiple times, we still need to zero the elevator at the start.
- The elevator is zeroed by a limit switch attached to trigger when the elevator is fully down.  To zero the elevator, the elevator moves downwards until it hits the limit switch, then resets the accumulated position as reported by the absolute encoder.
https://www.chiefdelphi.com/t/running-thrubore-on-an-elvator-shaft-and-trying-to-figure-out-wrapping-logic/456206/15
- The elevator is given a height to go to, and it uses PID control (on-motor if possible) to arrive at the height.  The PID controller also compensates for the extra weight of the elevator from gravity, potentially using Feedforward to compenate.

## End Effector Pivot
- Controlled by one NEO with a 25:1 gear ratio.
- The motor powers the pivot and angles the end effector.
- Feedback is gotten through an absolute encoder mounted on the pivot shaft.  This also means the mechanism doesn't have to zero at the beginning of the match.
- The mechanism is given a angle to go to, and it uses PID control to arrive at the angle.

## Actions

### Zeroing
When the robot is enabled, both sections of the mechanism should zero immediately.  The end effector pivot should move to a vertical position, and the elevator hsould be moved down slowly until it triggers the limit switch.
