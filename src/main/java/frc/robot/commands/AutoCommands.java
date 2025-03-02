package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.end_effector.EndEffectorSubsystem;
import frc.robot.subsystems.pivot.Pivot;

public class AutoCommands {

  public static Command manualScore(
      Drive drive,
      Elevator elevator,
      Pivot pivot,
      EndEffectorSubsystem endEffectorWheels,
      double height, double angle) {

    return DriveCommands.driveChassisSpeeds(drive, () -> new ChassisSpeeds(1, 0, 0))
        .withTimeout(4)
        .andThen(Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds())))
        .andThen(elevator.goToHeight(() -> height).withTimeout(2))
        .andThen(pivot.goToAngle(() -> angle).withTimeout(1))
        .andThen(
            Commands.run(
                    () -> {
                      System.out.println("Running wheels");
                      endEffectorWheels.setVelocity(0.5);
                    })
                .withTimeout(2))
        .andThen(
            Commands.runOnce(
                    () -> {
                      System.out.println("Stopping wheels");
                      endEffectorWheels.setVelocity(0);
                    })
                .andThen(pivot.goToAngle(() -> 0).withTimeout(1))
                .andThen(elevator.goToHeight(() -> 0)));
  }
}
