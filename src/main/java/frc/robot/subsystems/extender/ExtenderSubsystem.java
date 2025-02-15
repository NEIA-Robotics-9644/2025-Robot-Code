package frc.robot.subsystems.extender;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.extender.elevator.ElevatorIO;
import frc.robot.subsystems.extender.elevator.ElevatorIOInputsAutoLogged;
import frc.robot.subsystems.extender.pivot.PivotIO;
import frc.robot.subsystems.extender.pivot.PivotIOInputsAutoLogged;
import frc.robot.subsystems.extender.sensor.LimitSwitchSensorIO;
import frc.robot.subsystems.extender.sensor.LimitSwitchSensorIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class ExtenderSubsystem extends SubsystemBase {

  public enum ExtenderMode {
    HOMING,
    SETPOINT,
    MANUAL
  }

  private ExtenderMode mode = ExtenderMode.HOMING;

  int index = 0;

  double[] heights = {0, 0, 0, 0};
  double[] angles = {0, 0, 0, 0};

  private final ElevatorIO elevator;
  private final PivotIO pivot;
  private final LimitSwitchSensorIO limitSwitch;

  private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();
  private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();
  private final LimitSwitchSensorIOInputsAutoLogged limitSwitchInputs =
      new LimitSwitchSensorIOInputsAutoLogged();

  public ExtenderSubsystem(
      ElevatorIO elevator,
      PivotIO pivot,
      LimitSwitchSensorIO limitSwitch,
      double[] heights,
      double[] angles) {
    this.elevator = elevator;
    this.pivot = pivot;
    this.limitSwitch = limitSwitch;
    this.heights = heights;
    this.angles = angles;
  }

  @Override
  public void periodic() {
    elevator.updateInputs(elevatorInputs);
    pivot.updateInputs(pivotInputs);
    limitSwitch.updateInputs(limitSwitchInputs);
    elevator.periodic();
    pivot.periodic();
    Logger.processInputs("ElevatorExtender", elevatorInputs);
    Logger.processInputs("ElevatorPivot", pivotInputs);
    Logger.processInputs("LimitSwitch", limitSwitchInputs);

    Logger.recordOutput("ExtenderMode", mode.toString());

    Logger.recordOutput("HeightSetpoints", heights);
    Logger.recordOutput("AngleSetpoints", angles);

    switch (mode) {
      case HOMING:
        elevator.setManualVelocity(-0.02);
        if (limitSwitch.sensorHit()) {
          elevator.zeroEncoders();
          pivot.zeroEncoder();
          mode = ExtenderMode.SETPOINT;
        }

        break;
      case SETPOINT:
        elevator.setSetpointHeight(heights[index]);
        // pivot.setAngleSetpoint(angles[index]);
        break;

      case MANUAL:
        break;
    }
    if (limitSwitch.sensorHit()) {
      elevator.zeroEncoders();
      pivot.zeroEncoder();
    }
  }

  public void setElevatorVelocity(double velocity) {
    if (mode == ExtenderMode.HOMING) {
      System.out.println("Elevator is homing, cannot set velocity.");
      return;
    }
    elevator.setManualVelocity(velocity);
    mode = ExtenderMode.MANUAL;
  }

  public void setPivotVelocity(double velocity) {
    if (mode == ExtenderMode.HOMING) {
      System.out.println("Elevator is homing, cannot set velocity.");
      return;
    }
    pivot.setManualVelocity(velocity);
    mode = ExtenderMode.MANUAL;
  }

  public void setElevatorSetpoint(double height) {
    if (mode == ExtenderMode.HOMING) {
      System.out.println("Elevator is homing, cannot set velocity.");
      return;
    }
    elevator.setSetpointHeight(height);
    mode = ExtenderMode.SETPOINT;
  }

  public void setPivotSetpoint(double angle) {
    if (mode == ExtenderMode.HOMING) {
      System.out.println("Elevator is homing, cannot set velocity.");
      return;
    }
    pivot.setAngleSetpoint(angle);
    mode = ExtenderMode.SETPOINT;
  }

  public void setSetpoint(int index) {
    // Clamp
    if (index < 0) {
      index = 0;
    } else if (index > heights.length - 1) {
      index = heights.length - 1;
    }
    this.index = index;
    this.mode = ExtenderMode.SETPOINT;
  }

  public void modifySetpointHeight(double delta) {

    heights[index] += delta;
  }

  public void modifySetpointAngle(double delta) {

    angles[index] += delta;
  }
}
