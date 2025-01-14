![Static Badge](https://img.shields.io/badge/FRC-Team_9644-cb007b?style=flat-square&logo=first)
![Static Badge](https://img.shields.io/badge/WPI-Lib-%234d4848?style=flat-square&labelColor=8c150d)
![Static Badge](https://img.shields.io/badge/Photon-Vision-ffffff?style=flat-square&labelColor=006492)
![Static Badge](https://img.shields.io/badge/Path-Planner-ffffff?style=flat-square&labelColor=%232c3aad)
![Static Badge](https://img.shields.io/badge/Advantage-Scope-dee3ff?style=flat-square&labelColor=0027e6)
![Static Badge](https://img.shields.io/badge/Advantage-Kit-fff3d9?style=flat-square&labelColor=fec007)


# 2025 Robot Code
This is the code for our 2025 REEFSCAPE&trade; FRC robot.
We program our robot with Java, using WPILib.

## Code Structure
We use the command-based structure specified by WPILib, with some modifications.  We use Input-Output interfaces for any hardware-specific code to allow our subsystems to be fully hardware-independent and allow for full system simulation through the creation of mockup Input-Output interfaces.

## Git Protocol
The `main` branch should never contain broken or untested code, and should always be able to be deployed to the robot.
Any new features should be made on a branch of `main`, should be as small and well-defined as possible, and should be tested before completion.
When a new feature is complete, a pull request should be submitted to merge it to `main`, and all code should be reviewed before approval.
