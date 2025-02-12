package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.extender.ExtenderSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeCommands {

  // This command starts by making sure the extender is in the intake position
  // It runs the intake wheels and the end effector wheels until the end effector detects a coral
  public static Command intakeCoralFromStation(IntakeSubsystem intake, ExtenderSubsystem extender) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              System.out.println("Intaking coral from station");
              double elevatorDifference = Math.abs(extender.getElevatorAngle() - Constants.extenderAngles.get("Intake"));
              double effectorDifference = Math.abs(extender.getEffectorAngle() - Constants.effectorAngles.get("Intake"));
              if (intake.sensorState() && elevatorDifference < 0.25 && effectorDifference < 0.25) {
                intake.setVelocity(1);
              }
            }));
  }
}
