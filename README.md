# 2025 Robot Code
This is the code for our 2025 CRESCENDO FRC robot.
We program our robot with Java, using WPILib.

## Code Structure
We use the command-based structure specified by WPILib, with some modifications.  We use Input-Output interfaces for any hardware-specific code to allow our subsystems to be fully hardware-independent and allow for full system simulation through the creation of mockup Input-Output interfaces.

## Git Protocol
The `main` branch should never contain broken or untested code, and should always be able to be deployed to the robot.
Any new features should be made on a branch of `main`, should be as small and well-defined as possible, and should be tested before completion.
When a new feature is complete, a pull request should be submitted to merge it to `main`, and all code should be reviewed before approval.
