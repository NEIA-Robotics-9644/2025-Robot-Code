package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.controllers.AutoAlignController;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class AutoAlignCommands {

  // This was found by aligning the robot to tag 20 on the right side, then calculating the back and
  // side offsets from tag 20's position
  // https://www.geogebra.org/calculator/bsxrynbn

  public static final double NORMAL_TAG_BACKWARDS_OFFSET_METERS = 0.4652603933458 - 0.02;
  public static final double NORMAL_TAG_LEFT_OFFSET_METERS = -0.2013453599755 + 0.08;
  public static final double NORMAL_TAG_RIGHT_OFFSET_METERS = 0.2013453599755 - 0.002;

  public static final double NORMAL_TAG_ANGLE_OFFSET_RADIANS = Math.toRadians(180);

  private static List<Pose2d> getTargetPoses() {

    // Blue poses

    // First, get the poses of the april tags

    var tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    List<Integer> reefTags = List.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);
    List<Pose2d> tagPoses =
        tagLayout.getTags().stream()
            .filter(tag -> reefTags.contains(tag.ID))
            .map(
                tag ->
                    new Pose2d(
                        tag.pose.getX(),
                        tag.pose.getY(),
                        new Rotation2d(tag.pose.getRotation().getAngle())))
            .toList();

    Pose2d[] targetPoses = new Pose2d[24];

    // Then, add the offsets to the poses
    for (int i = 0; i < tagPoses.size(); i++) {
      var tagPose = tagPoses.get(i);
      var backwardsOffset = NORMAL_TAG_BACKWARDS_OFFSET_METERS;
      var angleOffset = NORMAL_TAG_ANGLE_OFFSET_RADIANS;

      var x =
          tagPose.getX()
              + backwardsOffset * Math.cos(tagPose.getRotation().getRadians())
              + NORMAL_TAG_LEFT_OFFSET_METERS * -Math.sin(tagPose.getRotation().getRadians());
      var y =
          tagPose.getY()
              + backwardsOffset * Math.sin(tagPose.getRotation().getRadians())
              + NORMAL_TAG_LEFT_OFFSET_METERS * Math.cos(tagPose.getRotation().getRadians());
      var rotation = new Rotation2d(tagPose.getRotation().getRadians() + angleOffset);

      targetPoses[i * 2] = new Pose2d(x, y, rotation);

      var x2 =
          tagPose.getX()
              + backwardsOffset * Math.cos(tagPose.getRotation().getRadians())
              + NORMAL_TAG_RIGHT_OFFSET_METERS * -Math.sin(tagPose.getRotation().getRadians());

      var y2 =
          tagPose.getY()
              + backwardsOffset * Math.sin(tagPose.getRotation().getRadians())
              + NORMAL_TAG_RIGHT_OFFSET_METERS * Math.cos(tagPose.getRotation().getRadians());

      targetPoses[i * 2 + 1] = new Pose2d(x2, y2, rotation);
    }

    return List.of(targetPoses);
  }

  // We will take the tag pose, move backwards by the backwards offset, move sideways by the
  // sideways offset, then rotate by the angle offset

  private static double distanceBetween(Pose2d pose1, Pose2d pose2) {
    return pose1.getTranslation().getDistance(pose2.getTranslation());
  }

  public static Pose2d findClosestPose(Pose2d currentPose, List<Pose2d> targetPoses) {
    if (targetPoses.isEmpty()) {
      System.out.println("No target poses available for comparison.");
      return currentPose; // No target poses to compare
    }

    Pose2d closestPose = targetPoses.get(0);
    double minDistance = 999999999;

    // Iterate through the list to find the closest pose
    for (Pose2d pose : targetPoses) {
      double distance = distanceBetween(currentPose, pose);
      if (distance < minDistance) {
        minDistance = distance;
        closestPose = pose;
      }
      System.out.println("Distance to pose: " + distance);
    }
    return closestPose;
  }

  private AutoAlignCommands() {}

  public static Command closestReefAlign(Drive drive) {

    final List<Pose2d> targetPoses;

    targetPoses = AutoAlignCommands.getTargetPoses();

    // Log the poses
    for (int i = 0; i < targetPoses.size(); i++) {
      var pose = targetPoses.get(i);
      Logger.recordOutput("Pose " + i, pose);
    }

    return new Command() {

      AutoAlignController alignController;

      @Override
      public void initialize() {

        alignController =
            new AutoAlignController(
                findClosestPose(drive.getPose(), targetPoses), null, false, drive);
      }

      @Override
      public void execute() {

        ChassisSpeeds speeds = alignController.update();
        ChassisSpeeds invertedSpeeds =
            new ChassisSpeeds(
                speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, -speeds.omegaRadiansPerSecond);
        drive.runVelocity(invertedSpeeds);
      }
    };
  }
}
