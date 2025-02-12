package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** All Constants Measured in Meters and Radians (m/s, m/s^2, rad/s, rad/s^2) */
/**
 * wheelRadius The radius of the wheels used in the drivetrain. trackWidthX The horizontal distance
 * between the left and right wheels of the drivetrain (also called the wheelbase width).
 * trackWidthY The vertical distance between the front and back wheels of the drivetrain (also
 * called the wheelbase length). bumperWidthX The total horizontal width of the robot, including the
 * bumpers. bumperWidthY The total vertical length of the robot, including the bumpers.
 * MaxLinearVelocity The maximum linear velocity the robot can achieve. maxLinearAcceleration The
 * maximum linear acceleration the robot can achieve. maxAngularVelocity The maximum angular
 * velocity (rotation speed) of the robot. rad/s maxAngularAcceleration The maximum angular
 * acceleration (change in rotation speed) of the robot. rad/s^2
 */
public final class DriveConstants {
  public static final DriveConfig driveConfig =
      new DriveConfig(
          Units.inchesToMeters(2),
          Units.inchesToMeters(22),
          Units.inchesToMeters(22),
          Units.inchesToMeters(31),
          Units.inchesToMeters(31),
          Units.feetToMeters(15.5),
          Units.feetToMeters(75.0),
          12.0,
          6.0);

  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(driveConfig.trackWidthX() / 2.0, driveConfig.trackWidthY() / 2.0),
        new Translation2d(driveConfig.trackWidthX() / 2.0, -driveConfig.trackWidthY() / 2.0),
        new Translation2d(-driveConfig.trackWidthX() / 2.0, driveConfig.trackWidthY() / 2.0),
        new Translation2d(-driveConfig.trackWidthX() / 2.0, -driveConfig.trackWidthY() / 2.0)
      };

  public static final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(moduleTranslations);

  public record DriveConfig(
      double wheelRadius,
      double trackWidthX,
      double trackWidthY,
      double bumperWidthX,
      double bumperWidthY,
      double maxLinearVelocity,
      double maxLinearAcceleration,
      double maxAngularVelocity,
      double maxAngularAcceleration) {
    public double driveBaseRadius() {
      return Math.hypot(trackWidthX / 2.0, trackWidthY / 2.0);
    }
  }

  public record ModuleConfig(
      int driveID,
      int turnID,
      int absoluteEncoderChannel,
      Rotation2d absoluteEncoderOffset,
      boolean turnMotorInverted) {}

  public record ModuleConstants(
      double ffkS,
      double ffkV,
      double ffkT,
      double drivekP,
      double drivekD,
      double turnkP,
      double turnkD,
      double driveReduction,
      double turnReduction) {}

  public record TrajectoryConstants(
      double linearkP,
      double linearkD,
      double thetakP,
      double thetakD,
      double linearTolerance,
      double thetaTolerance,
      double goalLinearTolerance,
      double goalThetaTolerance,
      double linearVelocityTolerance,
      double angularVelocityTolerance) {}

  public record HeadingControllerConstants(
      double kP, double kD, double maxVelocity, double maxAcceleration) {}
}
