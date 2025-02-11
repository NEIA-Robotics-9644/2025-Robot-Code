package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ExtenderCommands {

  public enum ExtenderSetpoint {
    L1,
    L2,
    L3,
    L4,
    Intake,
    Process,
    L2Dealgify,
    L3Dealgify,
  }

  public static Command goToSetpoint(ExtenderSetpoint setpoint, double timeout) {

    // This command moves the extender to a setpoint, which maps to a specified height and angle
    // This command ends when the extender reaches the setpoint

    return Commands.startEnd(
            () -> {
              System.out.println("Going to setpoint " + setpoint);
            },
            () -> {
              System.out.println("At setpoint: " + setpoint);
            })
        .withTimeout(timeout);
  }
}
