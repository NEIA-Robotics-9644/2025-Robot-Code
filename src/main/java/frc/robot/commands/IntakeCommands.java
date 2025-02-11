package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class IntakeCommands {

  // This command starts by making sure the extender is in the intake position
  // It runs the intake wheels and the end effector wheels until the end effector detects a coral
  public static Command intakeCoralFromStation() {
    return Commands.runOnce(
        () -> {
          System.out.println("Intaking coral from station");
        });
  }
}
