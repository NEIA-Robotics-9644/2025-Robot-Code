package frc.robot.subsystems.elevator.limit_sensor;

import org.littletonrobotics.junction.AutoLog;

public interface LimitSwitchSensorIO {

  @AutoLog
  static class LimitSwitchSensorIOInputs {
    public boolean sensorHit = false;
  }

  public default boolean sensorHit() {
    return true;
  }

  public default void updateInputs(LimitSwitchSensorIOInputs inputs) {}
}
