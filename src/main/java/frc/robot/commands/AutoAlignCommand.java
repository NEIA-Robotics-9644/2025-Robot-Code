package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.controllers.AutoAlignController;

public class AutoAlignCommand {

  public static Pose2d targetPose;

  private AutoAlignCommand() {}

  public static Command autoAlignCommandAprilTagCommand(Drive drive) {

    DriverStation.Alliance fieldSide =
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    AprilTagFieldLayout kTagLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();
    if (fieldSide == DriverStation.Alliance.Blue) {
      Pose2d tagPose = kTagLayout.getTagPose(1).orElse(null).toPose2d();
      Pose2d targetPose = tagPose;
    } else {
      Pose2d targetPose = null;
    }
    AutoAlignController alignController = new AutoAlignController(targetPose, null, false);

    return Commands.run(
        () -> {
          ChassisSpeeds speeds = alignController.update();
          drive.runVelocity(speeds);
        },
        drive);
  }
}
