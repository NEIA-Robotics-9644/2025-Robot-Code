package frc.robot.subsystems.extender.sensor;

public interface LimitSwitchSensorIO {
  public default boolean sensorHit() {
    return true;
  }
}
