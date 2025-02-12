package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.extender.ExtenderSubsystem;

public class EndEffectorCommands {

  public static Command scoreCoral() {
    // Spins the end effector wheels for a short amount of time to score a coral
    return Commands.runOnce(
        () -> {
          System.out.println("Scoring coral");
        });
  }

  public static Command dealgify() {
    // Spins the end effector wheels in reverse to dealgify a coral
    return Commands.runOnce(
        () -> {
          System.out.println("Dealgifying coral");
        });
  }
  public static Command setToPoint(ExtenderSubsystem extender, String position) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              System.out.println("Intaking coral from station");
              extender.moveEffectorToSetpoint(position);
            }));
  }
}
