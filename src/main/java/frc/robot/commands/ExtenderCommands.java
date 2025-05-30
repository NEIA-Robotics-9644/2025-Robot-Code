package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.util.ExtenderConstraints;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public class ExtenderCommands {

  private static final LoggedTunableNumber heightAtGoalTolerance;

  static {
    heightAtGoalTolerance = new LoggedTunableNumber("Extender/HeightAtGoalTolerance", 5.0);
  }

  public static Command goToHeightThenPivot(
      Elevator elevator,
      Pivot pivot,
      ExtenderConstraints constraints,
      DoubleSupplier inchesFromGround,
      DoubleSupplier degreesFromVertical) {
    return elevator
        .goToHeight(inchesFromGround)
        .alongWith(
            pivot.pivotConstraintsCommand(
                elevator::getHeightInches,
                constraints,
                () -> {
                  double elevatorError =
                      elevator.getHeightInches() - inchesFromGround.getAsDouble();
                  if (Math.abs(elevatorError) < heightAtGoalTolerance.getAsDouble()) {
                    return degreesFromVertical.getAsDouble();
                  } else {
                    return 0;
                  }
                }))
        .withName("Extender Movement With Constraints");
  }

  public static Command goToHeightNoConstraints(
    Elevator elevator,
    Pivot pivot,
    DoubleSupplier inchesFromGround,
    DoubleSupplier degreesFromVertical
  ) {
    return elevator
      .goToHeight(inchesFromGround),
      .alongWith(
        pivot.goToAngle(degreesFromVertical)
      ).withName("Extender Movement Without Constraints")
  }
}
