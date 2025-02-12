package frc.robot.subsystems.intake.sensor;

public interface CoralSensorIO {

  public default boolean coralDetected() {
    return true;
  }

  public default void setDisplayLight(boolean on) {}
}
