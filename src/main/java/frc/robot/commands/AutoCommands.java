package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.ControllerState;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.ReefSide;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.end_effector.EndEffectorSubsystem;

public class AutoCommands {

  public static Command autoScore(
      boolean slow,
      ReefSide side,
      Drive drive,
      EndEffectorSubsystem endEffector,
      ControllerState conState) {

    var delayUntilAutoAlign = 1 - 0.4;
    var delayUntilScore = 0.65 + (slow ? 1.5 : 0);
    var scoreDuration = 0.28;

    return conState
        .setSetpoint(conState.L4)
        .andThen(
            new WaitCommand(delayUntilAutoAlign)
                .andThen(
                    DriveCommands.joystickApproach(
                            drive,
                            () -> (slow ? 0.3 : 0.50),
                            () -> FieldConstants.getNearestReefBranch(drive.getPose(), side))
                        .withTimeout(delayUntilScore + scoreDuration)
                        .alongWith(
                            new WaitCommand(delayUntilScore)
                                .andThen(
                                    Commands.startEnd(
                                            () -> endEffector.setVelocity(1),
                                            () -> endEffector.setVelocity(0))
                                        .withTimeout(scoreDuration)))))
        .andThen(conState.setSetpoint(conState.INTAKE));
  }
}
