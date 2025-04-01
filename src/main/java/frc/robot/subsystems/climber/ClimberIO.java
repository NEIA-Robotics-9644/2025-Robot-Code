package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  static class ClimberIOInputs {
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double outputCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  default void updateInputs(ClimberIOInputs inputs) {}

  public default void setVelocity(double normalizedVelocity) {}

  public default double getPositionRads() {
    return 0.0;
  }

  public default void setMotorInverted(boolean inverted) {}

  public default void setMaxAmps(int maxAmps) {}

  public default void periodic() {}

  public default void zeroEncoder() {}
}
