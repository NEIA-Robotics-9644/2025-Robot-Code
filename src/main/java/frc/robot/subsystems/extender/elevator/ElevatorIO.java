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

    public ControlMode mode = ControlMode.MANUAL;

    public double targetHeight = 0.0;
    public double setpoint = 0.0;
    public double currentHeight = 0.0;
  }

  static class ElevatorIOConfig {
    public double kP = 0;
    public double kD = 0;
    public double kG = 0;
    public double maxSpeedUpNormalized = 0;
    public double maxSpeedDownNormalized = 0;
    public double maxHeight = 0;
    public int maxCurrentA = 0;
    public boolean leftReversed = false;
    public boolean rightReversed = false;

    public ElevatorIOConfig(
        double kP,
        double kD,
        double kG,
        double maxSpeedDownNormalized,
        double maxSpeedUpNormalized,
        double maxHeight,
        int maxCurrentA,
        boolean leftReversed,
        boolean rightReversed) {

      this.kP = kP;
      this.kD = kD;
      this.kG = kG;
      this.maxSpeedUpNormalized = maxSpeedUpNormalized;
      this.maxSpeedDownNormalized = maxSpeedDownNormalized;
      this.maxHeight = maxHeight;
      this.maxCurrentA = maxCurrentA;
      this.leftReversed = leftReversed;
      this.rightReversed = rightReversed;
    }
  }

  public enum ControlMode {
    SETPOINT,
    MANUAL
  }

  public default void periodic() {}

  public void configure(ElevatorIOConfig config);

  default void updateInputs(ElevatorIOInputs inputs) {}

  public void setManualVelocity(double normalizedVelocity);

  public void setSetpointHeight(double height);

  public void zeroEncoders();
}
