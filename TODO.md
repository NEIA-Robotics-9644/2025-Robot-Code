



# Rotation Issues

In simulation, auto-align and pathplanner-following is backwards.  On the real robot, it is not
THEORY: the swerve module rotation is flipped due to "rightSideInverted" and "leftSideInverted" being backwards in TunerConstants

TODO: Flip the swerve module rotation and flip the rotation in driveVelocity() in Drive.java