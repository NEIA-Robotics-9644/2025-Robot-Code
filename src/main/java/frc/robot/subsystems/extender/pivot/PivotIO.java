package frc.robot.subsystems.extender.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  @AutoLog
  static class PivotIOInputs {
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double outputCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  default void updateInputs(PivotIOInputs inputs) {}

  default void runVelocity(double velocity) {}

  default void setAngleSetpoint(String setpoint) {}

  public default void setManualVelocity(double normalizedVelocity) {}

  public default double getAngleDeg() {
    return 0.0;
  }

  public default double getVelocityPercent() {
    return 0.0;
  }

  public default void periodic() {}

  public default void setBrakeMode(boolean brake) {}
}
