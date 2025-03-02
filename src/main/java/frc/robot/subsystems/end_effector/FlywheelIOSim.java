package frc.robot.subsystems.end_effector;

import edu.wpi.first.math.geometry.Rotation2d;

public class FlywheelIOSim implements FlywheelIO {
  private double velocityRPM = 0.0;

  private double maxSpeedRPM = 5000.0;

  public double inputVelocity = 0.0;

  private boolean newInput = false;

  @Override
  public void setVelocity(double velocityPercent) {
    inputVelocity = maxSpeedRPM * velocityPercent;
    newInput = true;
  }

  @Override
  public double getVelocityPercent() {
    return velocityRPM / maxSpeedRPM;
  }

  @Override
  public void periodic() {

    if (newInput) {
      velocityRPM = inputVelocity;
      // System.out.println("New Velocity: " + velocityRPM);
      newInput = false;
    } else {
      velocityRPM = 0.0;
      // System.out.println("Reset Velocity: " + velocityRPM);
    }
  }

  @Override
  public void setBrakeMode(boolean brake) {
    // TODO Auto-generated method stub

  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.velocityRadPerSec = velocityRPM * (2 * Math.PI / 60);
    inputs.appliedVolts = 0.0;
    inputs.currentAmps = 0.0;
    inputs.tempCelsius = 0.0;
    inputs.relativeEncoderPosition = new Rotation2d(0.0);
    inputs.absoluteEncoderPosition = new Rotation2d(0.0);
  }
}
