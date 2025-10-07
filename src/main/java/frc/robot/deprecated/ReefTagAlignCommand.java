package frc.robot.deprecated;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public class ReefTagAlignCommand {

  public enum AlignGoal {
    LEFT,
    RIGHT,
    CENTER
  }

  /*
   * Auto-aligns to the visible tag using a closed-loop system
   */
  public static Command reefTagAlign(
      Drive drive, Vision vision, DoubleSupplier forwardSupplier, AlignGoal goal) {

    LoggedTunableNumber leftOffsetInches =
        new LoggedTunableNumber("ReefTagAlign/LeftOffsetInches", 12);
    LoggedTunableNumber rightOffsetInches =
        new LoggedTunableNumber("ReefTagAlign/RightOffsetInches", 12);

    LoggedTunableNumber leftRightP = new LoggedTunableNumber("ReefTagAlign/LeftRightP", 2);

    LoggedTunableNumber angleP = new LoggedTunableNumber("ReefTagAlign/AngleP", 0.4);

    // This should be reworked to not be an inline class creation
    return new Command() {

      Pose2d tagPose = new Pose2d();
      int tagId = 0;

      PIDController leftRightController = new PIDController(leftRightP.get(), 0, 0);
      PIDController angleController = new PIDController(angleP.get(), 0, 0);

      boolean seenTag = false;

      @Override
      public void initialize() {
        leftRightController.reset();
        angleController.reset();
        angleController.enableContinuousInput(-Math.PI, Math.PI);
        seenTag = false;
        tagId = 0;
      }

      @Override
      public void execute() {

        if (!seenTag) {
          var obs = vision.getNearestTagObservation();
          if (obs.isPresent() && VisionConstants.reefTags.contains(obs.get().tagId())) {
            tagPose = obs.get().tagOffset();
            tagId = obs.get().tagId();
            seenTag = true;
          }
        } else {

          var obs = vision.getTagObservation(tagId);
          if (obs.isPresent()) {
            tagPose = obs.get().tagOffset();
          }

          var forwards = forwardSupplier.getAsDouble();

          var targetLeftRightValue =
              switch (goal) {
                case LEFT -> Units.inchesToMeters(leftOffsetInches.get());
                case RIGHT -> -Units.inchesToMeters(rightOffsetInches.get());
                case CENTER -> 0;
              };

          var robotToTag = tagPose.minus(Pose2d.kZero);

          var robotFromTag = Pose2d.kZero.transformBy(robotToTag.inverse());

          var leftRight = leftRightController.calculate(robotFromTag.getY(), targetLeftRightValue);

          var angle = angleController.calculate(tagPose.getRotation().getRadians(), 0);

          var move =
              new Translation2d(forwards, leftRight)
                  .rotateBy(
                      VisionConstants.aprilTagLayout
                          .getTagPose(tagId)
                          .get()
                          .getRotation()
                          .toRotation2d());

          var rotate = drive.getPose().getRotation();

          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(move.getX(), move.getY(), -angle, rotate));
        }
      }
    };
  }
}
