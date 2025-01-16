// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.util.ModuleLimits;

/** All Constants Measured in Meters and Radians (m/s, m/s^2, rad/s, rad/s^2) */
/**
 * 
 * wheelRadius
 * The radius of the wheels used in the drivetrain.
 * trackWidthX
 * The horizontal distance between the left and right wheels of the drivetrain (also called the wheelbase width).
 * trackWidthY
 * The vertical distance between the front and back wheels of the drivetrain (also called the wheelbase length).
 * bumperWidthX
 * The total horizontal width of the robot, including the bumpers.
 * bumperWidthY
 * The total vertical length of the robot, including the bumpers.
 * MaxLinearVelocity
 * The maximum linear velocity the robot can achieve.
 * maxLinearAcceleration
 * The maximum linear acceleration the robot can achieve.
 * maxAngularVelocity
 * The maximum angular velocity (rotation speed) of the robot.
 * rad/s
 * maxAngularAcceleration
 * The maximum angular acceleration (change in rotation speed) of the robot.
 * rad/s^2
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

  // Odometry Constants
  public static final double odometryFrequency = 250.0;

  public static final Matrix<N3, N1> odometryStateStdDevs =
      new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.0002));

  // Module Constants
  public static final ModuleConfig[] moduleConfigs =
      new ModuleConfig[] {
        new ModuleConfig(16, 12, 0, new Rotation2d(-0.81761), true),
        new ModuleConfig(19, 14, 1, new Rotation2d(1.80875), true),
        new ModuleConfig(17, 13, 2, new Rotation2d(-0.48936), true),
        new ModuleConfig(18, 15, 3, new Rotation2d(-1.52578), true)
      };

  public static final ModuleConstants moduleConstants =
      new ModuleConstants(
          5.0,
          0.0,
          1.0 / DCMotor.getKrakenX60Foc(1).KtNMPerAmp, // A/(N*m)
          35.0,
          0.0,
          4000.0,
          50.0,
          Mk4iReductions.L3.reduction,
          Mk4iReductions.TURN.reduction);

  public static final ModuleLimits moduleLimitsFree =
      new ModuleLimits(
          driveConfig.maxLinearVelocity(),
          driveConfig.maxLinearAcceleration(),
          Units.degreesToRadians(1080.0));

  public static final ModuleLimits moduleLimitsFlywheelSpinup =
      new ModuleLimits(
          driveConfig.maxLinearVelocity(),
          driveConfig.maxLinearAcceleration() / 2.0,
          Units.degreesToRadians(1080.0));

  // Trajectory Following
  public static final TrajectoryConstants trajectoryConstants =
      new TrajectoryConstants(
          8.0,
          0.0,
          4.0,
          0.0,
          Units.inchesToMeters(4.0),
          Units.degreesToRadians(5.0),
          Units.inchesToMeters(5.0),
          Units.degreesToRadians(7.0),
          driveConfig.maxLinearVelocity() / 2.0,
          driveConfig.maxAngularVelocity() / 2.0);

  // Swerve Heading Control
  public static final HeadingControllerConstants headingControllerConstants =
      new HeadingControllerConstants(5.0, 0.0, 8.0, 20.0);

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

  private enum Mk4iReductions {
    L2((50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)),
    L3((50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0)),
    TURN((150.0 / 7.0));

    final double reduction;

    Mk4iReductions(double reduction) {
      this.reduction = reduction;
    }
  }
}
