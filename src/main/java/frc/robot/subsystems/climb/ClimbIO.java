package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {

  @AutoLog
  static class ClimbIOInputs {
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double outputCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  default void updateInputs(ClimbIOInputs inputs) {}

  public default void setVelocity(double normalizedVelocity) {}

  public default double getPositionRads() {
    return 0.0;
  }

  public default void setMaxAmps(int maxAmps) {}

  public default void setEncoderOffset(double encoderOffsetRads) {}

  public default void setInverted(boolean inverted) {}

  public default void periodic() {}

  public default void zeroEncoder() {}
}
