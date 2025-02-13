package frc.robot.subsystems.extender;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.extender.elevator.ElevatorIO;
import frc.robot.subsystems.extender.elevator.ElevatorIOInputsAutoLogged;
import frc.robot.subsystems.extender.pivot.PivotIO;
import frc.robot.subsystems.extender.pivot.PivotIOInputsAutoLogged;
import frc.robot.subsystems.extender.sensor.LimitSwitchSensorIO;
import org.littletonrobotics.junction.Logger;

public class ExtenderSubsystem extends SubsystemBase {
  private final ElevatorIO elevator;
  private final PivotIO pivot;
  private final LimitSwitchSensorIO limitSwitch;

  private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();
  private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();

  public ExtenderSubsystem(ElevatorIO elevator, PivotIO pivot, LimitSwitchSensorIO limitSwitch) {
    this.elevator = elevator;
    this.pivot = pivot;
    this.limitSwitch = limitSwitch;
  }

  @Override
  public void periodic() {
    elevator.updateInputs(elevatorInputs);
    pivot.updateInputs(pivotInputs);
    elevator.periodic();
    pivot.periodic();
    Logger.processInputs("ElevatorExtender", elevatorInputs);
    Logger.processInputs("ElevatorPivot", pivotInputs);

    if (elevator.atBottom() && limitSwitch.sensorHit()) {
      elevator.resetAngleToBottom();
    }
  }

  public void setElevatorVelocity(double velocity) {
    elevator.setManualVelocity(velocity);
  }

  public void setPivotVelocity(double velocity) {
    pivot.setManualVelocity(velocity);
  }

  public void moveElevatorToSetpoint(String position) {
    elevator.setAngleSetpoint(position);
  }

  public void moveEffectorToSetpoint(String position) {
    pivot.setAngleSetpoint(position);
  }

  public double getElevatorAngle() {
    return elevator.getAngleDeg();
  }

  public double getEffectorAngle() {
    return pivot.getAngleDeg();
  }
}
