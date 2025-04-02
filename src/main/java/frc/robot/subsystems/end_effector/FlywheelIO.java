package frc.robot.subsystems.end_effector;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  class FlywheelIOInputs {
    public Rotation2d relativeEncoderPosition = new Rotation2d();
    public Rotation2d absoluteEncoderPosition = new Rotation2d();
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  default void updateInputs(FlywheelIOInputs inputs) {}

  public default void setBrakeMode(boolean brake) {}

  public default double getVelocityPercent() {
    return 0.0;
  }

  default void stop() {}

  public default void setVelocity(double normalizedVelocity) {}

  public default double getPositionRads() {
    return 0.0;
  }

  public default void setMaxAmps(int maxAmps) {}

  public default void setEncoderOffset(double encoderOffsetRads) {}

  public default void setInverted(boolean inverted) {}

  public default void zeroEncoder() {}

  public default void periodic() {}
}
