package frc.robot.subsystems.intake.wheel;

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

  default void runVelocity(double velocity) {}

  default public void setVelocity(double normalizedVelocity){}

  default public double getVelocityPercent(){return 0.0;}

  default public void periodic(){}

  default public void setBrakeMode(boolean brake){}
}
