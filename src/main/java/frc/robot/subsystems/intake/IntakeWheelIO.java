package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeWheelIO {
  @AutoLog
  static class IntakeWheelIOInputs {
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double outputCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  default void updateInputs(IntakeWheelIOInputs inputs) {}

  public default void setVelocity(double normalizedVelocity) {}

  public default double getVelocityPercent() {
    return 0.0;
  }

  public default void periodic() {}

  public default void setBrakeMode(boolean brake) {}
}
