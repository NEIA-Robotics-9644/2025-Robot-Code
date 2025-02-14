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

    public double currentAngleDeg = 0.0;
    public double targetAngleDeg = 0.0;
  }

  default void updateInputs(PivotIOInputs inputs) {}

  default void setAngleSetpoint(double angleDeg) {}

  public default void setManualVelocity(double normalizedVelocity) {}

  public default void periodic() {}

  public default void zeroEncoder() {}
}
