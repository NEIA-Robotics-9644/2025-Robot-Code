package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.controllers.AutoAlignController;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class AutoAlignCommands {

  private static double distanceBetween(Pose2d pose1, Pose2d pose2) {
    return pose1.getTranslation().getDistance(pose2.getTranslation());
  }

  public static Pose2d findClosestPose(Pose2d currentPose, List<Pose2d> targetPoses) {
    if (targetPoses.isEmpty()) {
      return currentPose; // No target poses to compare
    }

    Pose2d closestPose = targetPoses.get(0);
    double minDistance = distanceBetween(currentPose, closestPose);

    // Iterate through the list to find the closest pose
    for (Pose2d pose : targetPoses) {
      double distance = distanceBetween(currentPose, pose);
      if (distance < minDistance) {
        minDistance = distance;
        closestPose = pose;
      }
    }
    return closestPose;
  }

  private AutoAlignCommands() {}

  public static Command closestReefAlign(Drive drive) {

    Pose2d currentPose = drive.getPose();
    List<Pose2d> targetPoses = List.of();

    DriverStation.Alliance fieldSide =
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    AprilTagFieldLayout kTagLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();
    if (fieldSide == DriverStation.Alliance.Blue) { // blue alliance reef autoalign
      targetPoses =
          List.of(
              new Pose2d(5.716, 4.197, new Rotation2d(Math.toRadians(180))), // H
              new Pose2d(5.716, 3.861, new Rotation2d(Math.toRadians(180))), // G
              new Pose2d(5.239, 3.040, new Rotation2d(Math.toRadians(120))), // F
              new Pose2d(4.966, 2.877, new Rotation2d(Math.toRadians(120))), // E
              new Pose2d(4.028, 2.877, new Rotation2d(Math.toRadians(60))), // D
              new Pose2d(3.742, 3.040, new Rotation2d(Math.toRadians(60))), // C
              new Pose2d(3.270, 3.850, new Rotation2d(Math.toRadians(0))), // B
              new Pose2d(3.270, 4.197, new Rotation2d(Math.toRadians(0))), // A
              new Pose2d(3.742, 4.943, new Rotation2d(Math.toRadians(-60))), // L
              new Pose2d(4.028, 5.167, new Rotation2d(Math.toRadians(-60))), // K
              new Pose2d(4.966, 5.167, new Rotation2d(Math.toRadians(-120))), // J
              new Pose2d(5.239, 4.943, new Rotation2d(Math.toRadians(-120))) // I
              );
    } else if (fieldSide == DriverStation.Alliance.Red) {

    }

    for (int i = 0; i < targetPoses.size(); i++) {
      Logger.recordOutput("Pose " + i, targetPoses.get(i));
    }

    AutoAlignController alignController =
        new AutoAlignController(findClosestPose(currentPose, targetPoses), null, true, drive);

    return Commands.run(
        () -> {
          ChassisSpeeds speeds = alignController.update();
          drive.runVelocity(speeds);
        },
        drive);
  }
}
