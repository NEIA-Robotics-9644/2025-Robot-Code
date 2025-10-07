package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ControllerState;
import frc.robot.ControllerState.ExtenderSetpoint;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.commands.ExtenderCommands;


public class AlgaeCommands {

    public static Command AlgaeIntake(
        ControllerState controller, 
        ExtenderSetpoint AlgaeLevelIntake, 
        ExtenderSetpoint AlgaeLevelHold, 
        Trigger intakeComplete, 
        Intake intake) {

        return Commands.sequence(
            Commands.runOnce(() -> controller.setSetpoint(AlgaeLevelIntake)),
            Commands.runEnd(
                () -> intake.setVelocity(0.5),
                () -> intakeComplete.onTrue(Commands.runOnce(() -> intake.setVelocity(0)))
            ),
            Commands.runOnce(() -> controller.setSetpoint(AlgaeLevelHold))
        );

    }

    // public static Command AlgaeScore(
    //     ControllerState controller,
    //     ExtenderSetpoint AlgaeLevelScore,
    //     Trigger score,
    //     Intake intake,
    //     Elevator elevator,
    //     Pivot pivot,
    //     DoubleSupplier AlgaeLevelScoreHeight,
    //     DoubleSupplier AlgaeLevelScoreAngle
    // ) {


    //     return Commands.sequence(
    //         Commands.runOnce(ExtenderCommands.goToHeightNoConstraints(elevator, pivot, AlgaeLevelScoreHeight, AlgaeLevelScoreAngle)),
    //         Commands.runOnce(
    //         () -> score.onTrue(Commands.runOnce(() -> intake.setVelocity(-0.5))))    
            

    //     );

    // }
}
