package frc.robot.subsystems.climber;

public class ClimberIOSim implements ClimberIO {

  private double positionRads = 0.0;
  private double velocityRadsPerSec = 0.0;

  @Override
  public void periodic() {
    positionRads += velocityRadsPerSec * 0.02; // Simulate 20ms periodic update
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.positionRads = positionRads;
    inputs.velocityRadsPerSec = velocityRadsPerSec;
    inputs.appliedVoltage = 0.0; // No voltage in simulation
    inputs.outputCurrentAmps = 0.0; // No current in simulation
    inputs.tempCelsius = 0.0; // No temperature in simulation
  }

  @Override
  public void setMaxAmps(int maxAmps) {
    // No-op for simulation
  }

  @Override
  public void setVelocity(double normalizedVelocity) {
    this.velocityRadsPerSec = normalizedVelocity * 100;
  }

  @Override
  public double getPositionRads() {
    return positionRads;
  }

  @Override
  public void zeroEncoder() {
    positionRads = 0.0;
  }
}
