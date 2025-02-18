package frc.robot.subsystems.elevator.elevator_motors;

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

  default void updateInputs(ElevatorIOInputs elevatorIOInputs) {}

  public void setVelocity(double normalizedVelocity);

  public void setMaxAmps(int maxAmps);

  public double getPositionRads();

  public void zeroEncoders();
}
