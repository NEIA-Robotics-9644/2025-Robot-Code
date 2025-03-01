package frc.robot.subsystems.drive.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.poseEstimator.Vision;
import java.util.function.Supplier;

public class AutoAlignController {
  private static final double linearkP = 3.5;
  private static final double linearkD = 0;
  private static final double thetakP = 1.4;
  private static final double thetakD = 0;
  private static final double linearTolerance = 0.08;
  private static final double thetaTolerance = Units.degreesToRadians(2.0);
  private static final double toleranceTime = 0.5;
  private static final double maxLinearVelocity =
      DriveConstants.driveConfig.maxLinearVelocity() * 0.5;
  private static final double maxLinearAcceleration =
      DriveConstants.driveConfig.maxLinearAcceleration() * 0.4;
  private static final double maxAngularVelocity =
      DriveConstants.driveConfig.maxAngularVelocity() * 0.8;
  private static final double maxAngularAcceleration =
      DriveConstants.driveConfig.maxAngularAcceleration() * 0.8;
  private static final double slowLinearVelocity = 2.25;
  private static final double slowLinearAcceleration = 3.0;
  private static final double slowAngularVelocity = Math.PI / 2.0;
  private static final double slowAngularAcceleration = Math.PI;
  private static final double ffMinRadius = 0.2;
  private static final double ffMaxRadius = 0.8;

  public Vision vision;
  public Drive drive;

  private final Pose2d poseSupplier;
  private final boolean slowMode;
  private Translation2d lastSetpointTranslation;

  // Controllers for translation and rotation
  private final ProfiledPIDController linearController;
  private final ProfiledPIDController thetaController;
  private final Timer toleranceTimer = new Timer();

  public AutoAlignController(
      Pose2d poseSupplier,
      Supplier<Translation2d> feedforwardSupplier,
      boolean slowMode,
      Drive drive) {
    this.poseSupplier = poseSupplier;
    this.slowMode = slowMode;
    this.drive = drive;

    // Set up both controllers
    linearController =
        new ProfiledPIDController(linearkP, 0, linearkD, new TrapezoidProfile.Constraints(0, 0));
    linearController.setTolerance(linearTolerance);
    thetaController =
        new ProfiledPIDController(thetakP, 0, thetakD, new TrapezoidProfile.Constraints(0, 0));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(thetaTolerance);
    toleranceTimer.restart();
    updateConstraints();
    resetControllers();
  }

  private void updateConstraints() {
    if (slowMode) {
      linearController.setConstraints(
          new TrapezoidProfile.Constraints(slowLinearVelocity, slowLinearAcceleration));
      thetaController.setConstraints(
          new TrapezoidProfile.Constraints(slowAngularVelocity, slowAngularAcceleration));
    } else {
      linearController.setConstraints(
          new TrapezoidProfile.Constraints(maxLinearVelocity, maxLinearAcceleration));
      thetaController.setConstraints(
          new TrapezoidProfile.Constraints(maxAngularVelocity, maxAngularAcceleration));
    }
  }

  private void resetControllers() {
    Pose2d currentPose = frc.robot.RobotState.getInstance().getEstimatedPose(drive);
    Pose2d goalPose = poseSupplier;
    Twist2d fieldVelocity = frc.robot.RobotState.getInstance().getFieldTwist();
    Rotation2d robotToGoalAngle =
        goalPose.getTranslation().minus(currentPose.getTranslation()).getAngle();
    double linearVelocity =
        Math.min(
            0.0,
            -new Translation2d(fieldVelocity.dx, fieldVelocity.dy)
                .rotateBy(robotToGoalAngle.unaryMinus())
                .getX());
    linearController.reset(
        currentPose.getTranslation().getDistance(goalPose.getTranslation()), linearVelocity);
    thetaController.reset(currentPose.getRotation().getRadians(), fieldVelocity.dtheta);
    lastSetpointTranslation = currentPose.getTranslation();
  }

  public ChassisSpeeds update() {
    Pose2d currentPose = frc.robot.RobotState.getInstance().getEstimatedPose(drive);
    Pose2d targetPose = poseSupplier;

    double currentDistance =
        currentPose.getTranslation().getDistance(poseSupplier.getTranslation());
    double ffScaler =
        MathUtil.clamp((currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius), 0.0, 1.0);
    linearController.reset(
        lastSetpointTranslation.getDistance(targetPose.getTranslation()),
        linearController.getSetpoint().velocity);
    double driveVelocityScalar =
        linearController.getSetpoint().velocity * ffScaler
            + linearController.calculate(currentDistance, 0.0);
    if (linearController.atGoal()) driveVelocityScalar = 0.0;
    lastSetpointTranslation =
        new Pose2d(
                targetPose.getTranslation(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(
                frc.robot.util.GeomUtil.toTransform2d(linearController.getSetpoint().position, 0.0))
            .getTranslation();

    double thetaVelocity =
        thetaController.getSetpoint().velocity * ffScaler
            + thetaController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    if (thetaController.atGoal()) thetaVelocity = 0.0;

    if (!linearController.atGoal() || !thetaController.atGoal()) {
      toleranceTimer.reset();
    }

    var driveVelocity =
        new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(frc.robot.util.GeomUtil.toTransform2d(driveVelocityScalar, 0.0))
            .getTranslation();
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation());
  }

  public boolean atGoal() {
    return toleranceTimer.hasElapsed(toleranceTime);
  }
}
