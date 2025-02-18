package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.elevator.elevator_motors.ElevatorIO;
import frc.robot.subsystems.elevator.elevator_motors.ElevatorIOInputsAutoLogged;
import frc.robot.subsystems.elevator.limit_sensor.LimitSwitchSensorIO;
import frc.robot.subsystems.elevator.limit_sensor.LimitSwitchSensorIOInputsAutoLogged;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private LoggedTunableNumber elevatorP = new LoggedTunableNumber("Elevator/P", 0.3);
  private LoggedTunableNumber elevatorI = new LoggedTunableNumber("Elevator/I", 0.0);
  private LoggedTunableNumber elevatorD = new LoggedTunableNumber("Elevator/D", 0.0);

  // The constant power needed to keep the elevator at rest
  private LoggedTunableNumber elevatorG = new LoggedTunableNumber("Elevator/G", 0.12);

  private LoggedTunableNumber elevatorMaxSpeedUp =
      new LoggedTunableNumber("Elevator/MaxSpeedUp", 1);
  private LoggedTunableNumber elevatorMaxSpeedDown =
      new LoggedTunableNumber("Elevator/MaxSpeedDown", 0.01);

  private LoggedTunableNumber elevatorMaxHeight = new LoggedTunableNumber("Elevator/MaxHeight", 12);
  private LoggedTunableNumber elevatorHomingMoveSpeed =
      new LoggedTunableNumber("Elevator/HomingMoveSpeed", 0.1);
  private LoggedTunableNumber elevatorMaxAmps =
      new LoggedTunableNumber("Elevator/MaxAmpsPerMotor", 60);

  private final ElevatorIO elevatorIO;
  private final ElevatorIOInputsAutoLogged elevatorIOInputs = new ElevatorIOInputsAutoLogged();
  private final LimitSwitchSensorIO limitSwitchSensorIO;
  private final LimitSwitchSensorIOInputsAutoLogged limitSwitchSensorIOInputs =
      new LimitSwitchSensorIOInputsAutoLogged();

  private final PIDController feedback = new PIDController(0, 0, 0);

  private boolean isHomed = false;

  public Elevator(ElevatorIO elevatorIO, LimitSwitchSensorIO limitSwitchSensorIO) {
    this.elevatorIO = elevatorIO;
    this.limitSwitchSensorIO = limitSwitchSensorIO;
  }

  @Override
  public void periodic() {
    elevatorIO.updateInputs(elevatorIOInputs);
    elevatorIO.periodic();
    limitSwitchSensorIO.updateInputs(limitSwitchSensorIOInputs);
    elevatorIO.setMaxAmps((int) elevatorMaxAmps.get());

    var currentCommand = getCurrentCommand();

    String loggedName = currentCommand != null ? currentCommand.getName() : "None";

    Logger.recordOutput("Elevator/CurrentCommand", loggedName);
    Logger.recordOutput("Elevator/Homed", isHomed);
  }

  @AutoLogOutput(key = "Elevator/CurrentPositionRads")
  public double getPosition() {
    return elevatorIO.getPositionRads();
  }

  private void setVelocity(double normalizedVelocity) {
    elevatorIO.setVelocity(normalizedVelocity);
  }

  private void resetElevator() {
    elevatorIO.zeroEncoders();
    isHomed = true;
  }

  private boolean limitSwitchTripped() {
    return limitSwitchSensorIO.sensorHit();
  }

  public Trigger isHomed() {
    return new Trigger(() -> isHomed);
  }

  // Commands

  /**
   * Runs the pid loop which moves the elevator to the right height. Don't end the command
   * prematurely
   */
  public Command goToHeight(DoubleSupplier normalizedHeight) {

    var command =
        Commands.runEnd(
                () -> {
                  Logger.recordOutput(
                      "Elevator/TargetPositionRads",
                      normalizedHeight.getAsDouble() * elevatorMaxHeight.get());
                  feedback.setPID(elevatorP.get(), elevatorI.get(), elevatorD.get());

                  var output =
                      feedback.calculate(
                              elevatorIO.getPositionRads(),
                              normalizedHeight.getAsDouble() * elevatorMaxHeight.get())
                          + elevatorG.get();

                  if (output > 0) {
                    output = Math.min(output, elevatorMaxSpeedUp.get());
                  } else {
                    output = Math.max(output, -elevatorMaxSpeedDown.get());
                  }
                  setVelocity(output);
                },
                () -> {
                  setVelocity(0);
                },
                this)
            .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
            .withName("Elevator Setpoint Control");

    return command;
  }

  public Command manualControl(DoubleSupplier manualSpeed) {
    var command =
        Commands.runEnd(() -> setVelocity(manualSpeed.getAsDouble()), () -> setVelocity(0), this)
            .withName("Manual Elevator Control")
            .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    return command;
  }

  /**
   * This should be called before the elevator is used. It will move the elevator down until the
   * limit switch is hit and then zero the encoders
   */
  public Command home() {
    var command =
        Commands.run(() -> elevatorIO.setVelocity(-elevatorHomingMoveSpeed.get()), this)
            .until(() -> limitSwitchTripped())
            .andThen(Commands.runOnce(() -> resetElevator()))
            .withName("Homing Sequence")
            .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    return command;
  }
}
