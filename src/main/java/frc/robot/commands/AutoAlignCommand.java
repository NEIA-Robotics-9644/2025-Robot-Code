package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.controllers.AutoAlignController;
import frc.robot.subsystems.poseEstimator.Vision;
import java.util.function.Supplier;

public class AutoAlignCommand {

  private AutoAlignCommand() {}

  public static Command autoAlignCommandAprilTagCommand(
      Supplier<Pose2d> visionPoseSupplier, Drive drive, Vision vision) {

    AutoAlignController alignController = new AutoAlignController(visionPoseSupplier, null, false);

    return Commands.run(
        () -> {
          ChassisSpeeds speeds = alignController.update(vision);
          drive.runVelocity(speeds);
        },
        drive);
  }
}
