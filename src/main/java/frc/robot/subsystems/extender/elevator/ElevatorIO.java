package frc.robot.subsystems.extender.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  static class ElevatorIOInputs {
    public double positionRadsL = 0.0;
    public double velocityRadsPerSecL = 0.0;
    public double appliedVoltageL = 0.0;
    public double outputCurrentAmpsL = 0.0;
    public double tempCelsiusL = 0.0;

    public double positionRadsR = 0.0;
    public double velocityRadsPerSecR = 0.0;
    public double appliedVoltageR = 0.0;
    public double outputCurrentAmpsR = 0.0;
    public double tempCelsiusR = 0.0;
  }

  public default void periodic() {}

  default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setAngleSetpoint(String height) {}

  public default double getAngleDeg() {
    return 0.0;
  }

  /*
   * Enable manual control of the mechanism
   * This disables all internal control loops
   * USE WITH CAUTION
   */
  public default void setManualControl(boolean enabled) {}

  /*
   * Set the velocity of the mechanism
   * Negative is always down.
   * This is only used when manual control is enabled
   */
  public default void setManualVelocity(double velocity) {}

  /*
   * Get whether manual control is enabled
   */
  public default boolean manualControlEnabled() {
    return true;
  }

  public default boolean atBottom() {
    return true;
  }

  public default boolean atTop() {
    return true;
  }

  public default void resetAngleToBottom() {}

  public default double getTopAngleDeg() {
    return 0.0;
  }

  public default double getBottomAngleDeg() {
    return 0.0;
  }

  public default double getVelocityPercent() {
    return 0.0;
  }
}
