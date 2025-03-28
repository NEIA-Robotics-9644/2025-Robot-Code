package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Alert;
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

  private LoggedTunableNumber elevatorP = new LoggedTunableNumber("Elevator/P", 0.1);
  private LoggedTunableNumber elevatorI = new LoggedTunableNumber("Elevator/I", 0.0);
  private LoggedTunableNumber elevatorD = new LoggedTunableNumber("Elevator/D", 0.0);

  // The constant power needed to keep the elevator at rest
  private LoggedTunableNumber elevatorG = new LoggedTunableNumber("Elevator/G", 0.1);

  private LoggedTunableNumber elevatorMaxSpeedUp =
      new LoggedTunableNumber("Elevator/MaxSpeedUp", 1);
  private LoggedTunableNumber elevatorMaxSpeedDown =
      new LoggedTunableNumber("Elevator/MaxSpeedDown", 0.23);

  private LoggedTunableNumber elevatorMaxHeightRadians =
      new LoggedTunableNumber("Elevator/MaxHeightRadians", 21.166568756103516 + 0.1);

  // How far the elevator (measured from the end effector pivot axis) is off the ground when the
  // elevator is all the way down
  private LoggedTunableNumber elevatorDownHeightInches =
      new LoggedTunableNumber("Elevator/DownHeightInches", 12.5);

  // How far the elevator (measured from the end effector pivot axis) is off the ground when the
  // elevator is all the way up
  private LoggedTunableNumber elevatorUpHeightInches =
      new LoggedTunableNumber("Elevator/UpHeightInches", 80);

  private LoggedTunableNumber elevatorHomingMoveSpeed =
      new LoggedTunableNumber("Elevator/HomingMoveSpeed", 0.05);
  private LoggedTunableNumber elevatorMaxAmps =
      new LoggedTunableNumber("Elevator/MaxAmpsPerMotor", 80);

  private static final double MAX_OK_TEMP_CELSIUS = 90.0; // degrees Celsius

  private Alert rightMotorOverheatingAlert =
      new Alert(
          "The Right Elevator Motor is over "
              + MAX_OK_TEMP_CELSIUS
              + " deg C.  Give the robot time to cool down.",
          Alert.AlertType.kWarning);

  private Alert leftMotorOverheatingAlert =
      new Alert(
          "The Left Elevator Motor is over "
              + MAX_OK_TEMP_CELSIUS
              + " deg C.  Give the robot time to cool down.",
          Alert.AlertType.kWarning);

  private final ElevatorIO elevatorIO;
  private final ElevatorIOInputsAutoLogged elevatorIOInputs = new ElevatorIOInputsAutoLogged();
  private final LimitSwitchSensorIO limitSwitchSensorIO;
  private final LimitSwitchSensorIOInputsAutoLogged limitSwitchSensorIOInputs =
      new LimitSwitchSensorIOInputsAutoLogged();

  private final PIDController feedback = new PIDController(0, 0, 0);

  private boolean isHomed = false;

  public double inchesHeightToRadians(double inches) {
    return (inches - elevatorDownHeightInches.get())
        / (elevatorUpHeightInches.get() - elevatorDownHeightInches.get())
        * elevatorMaxHeightRadians.get();
  }

  public double radiansToInchesHeight(double radians) {
    return (radians / elevatorMaxHeightRadians.get())
            * (elevatorUpHeightInches.get() - elevatorDownHeightInches.get())
        + elevatorDownHeightInches.get();
  }

  public double normalizedPositionToInchesHeight(double normalizedPosition) {
    var height =
        (normalizedPosition * (elevatorUpHeightInches.get() - elevatorDownHeightInches.get()))
            + elevatorDownHeightInches.get();

    return MathUtil.clamp(height, elevatorDownHeightInches.get(), elevatorUpHeightInches.get());
  }

  public double inchesHeightToNormalizedPosition(double inches) {
    var normalizedPosition =
        (inches - elevatorDownHeightInches.get())
            / (elevatorUpHeightInches.get() - elevatorDownHeightInches.get());

    return MathUtil.clamp(normalizedPosition, 0, 1);
  }

  public Elevator(ElevatorIO elevatorIO, LimitSwitchSensorIO limitSwitchSensorIO) {
    this.elevatorIO = elevatorIO;
    this.limitSwitchSensorIO = limitSwitchSensorIO;
  }

  private int oldMaxAmps = 0;

  @Override
  public void periodic() {
    elevatorIO.updateInputs(elevatorIOInputs);
    Logger.processInputs("ElevatorMotors", elevatorIOInputs);
    elevatorIO.periodic();
    limitSwitchSensorIO.updateInputs(limitSwitchSensorIOInputs);
    Logger.processInputs("ElevatorLimitSwitch", limitSwitchSensorIOInputs);

    if (oldMaxAmps != (int) elevatorMaxAmps.get()) {
      elevatorIO.setMaxAmps((int) elevatorMaxAmps.get());
      oldMaxAmps = (int) elevatorMaxAmps.get();
    }

    var currentCommand = getCurrentCommand();

    String loggedName = currentCommand != null ? currentCommand.getName() : "None";

    Logger.recordOutput("Elevator/CurrentCommand", loggedName);
    Logger.recordOutput("Elevator/Homed", isHomed);

    leftMotorOverheatingAlert.set(elevatorIOInputs.tempCelsiusL > MAX_OK_TEMP_CELSIUS);
    rightMotorOverheatingAlert.set(elevatorIOInputs.tempCelsiusR > MAX_OK_TEMP_CELSIUS);
  }

  @AutoLogOutput(key = "Elevator/CurrentPositionInches")
  public double getHeightInches() {
    return radiansToInchesHeight(elevatorIO.getPositionRads());
  }

  public void setVelocity(double normalizedVelocity) {
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
   * prematurely The height in inches is measured from the ground to the end effector pivot axis
   */
  public Command goToHeight(DoubleSupplier heightInches) {

    var command =
        Commands.runEnd(
                () -> {
                  Logger.recordOutput("Elevator/TargetPositionInches", heightInches.getAsDouble());
                  feedback.setPID(elevatorP.get(), elevatorI.get(), elevatorD.get());

                  var output =
                      feedback.calculate(
                              elevatorIO.getPositionRads(),
                              this.inchesHeightToRadians(heightInches.getAsDouble()))
                          + elevatorG.get();

                  if (output > 0) {
                    output = Math.min(output, elevatorMaxSpeedUp.get());
                  } else {
                    output = Math.max(output, -elevatorMaxSpeedDown.get());
                  }

                  output =
                      MathUtil.clamp(output, -elevatorMaxSpeedDown.get(), elevatorMaxSpeedUp.get());

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

  public double getMinInchesFromGround() {
    return elevatorDownHeightInches.get();
  }

  public double getMaxInchesFromGround() {
    return elevatorUpHeightInches.get();
  }
}
