package frc.robot.subsystems.elevator.elevator_motors;

public class ElevatorIOSim implements ElevatorIO {

  private final double MAX_MOTOR_SPEED_RADS = 100;
  private double motorVelocityRads = 0.0;
  private double motorPositionRads = 0.0;

  @Override
  public void setVelocity(double normalizedVelocity) {
    // This isn't a very good simulation, but it works for now
    motorVelocityRads = normalizedVelocity * MAX_MOTOR_SPEED_RADS;
  }

  @Override
  public void periodic() {
    motorPositionRads += motorVelocityRads * 0.02;
  }

  @Override
  public double getPositionRads() {
    return motorPositionRads;
  }

  @Override
  public void zeroEncoders() {
    motorPositionRads = 0.0;
  }

  @Override
  public void setMaxAmps(int maxAmps) {
    // No-op for simulation
  }

  @Override
  public void updateInputs(ElevatorIOInputs elevatorIOInputs) {
    elevatorIOInputs.positionRadsL = motorPositionRads;
    elevatorIOInputs.velocityRadsPerSecL = motorVelocityRads;
    elevatorIOInputs.appliedVoltageL = 0.0;
    elevatorIOInputs.outputCurrentAmpsL = 0.0;
    elevatorIOInputs.tempCelsiusL = 0.0;

    elevatorIOInputs.positionRadsR = motorPositionRads;
    elevatorIOInputs.velocityRadsPerSecR = motorVelocityRads;
    elevatorIOInputs.appliedVoltageR = 0.0;
    elevatorIOInputs.outputCurrentAmpsR = 0.0;
    elevatorIOInputs.tempCelsiusR = 0.0;
  }
}
