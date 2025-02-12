package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.extender.ExtenderSubsystem;

public class ExtenderCommands {

    public static Command setToPoint(ExtenderSubsystem extender, String position) {
      return Commands.sequence(
          Commands.runOnce(
              () -> {
                System.out.println("Intaking coral from station");
                extender.moveElevatorToSetpoint(position);
              }));
    }
}
