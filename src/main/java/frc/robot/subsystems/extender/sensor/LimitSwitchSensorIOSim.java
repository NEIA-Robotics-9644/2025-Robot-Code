package frc.robot.subsystems.extender.sensor;

public class LimitSwitchSensorIOSim implements LimitSwitchSensorIO {
  private boolean hit = false;

  // Simulate a simple sensor
  // Use an input from SmartDashboard to simulate the sensor

  public LimitSwitchSensorIOSim() {
    // Set up the SmartDashboard input
    // SmartDashboard.putBoolean("Coral Detected Sim Input", false);
  }

  @Override
  public boolean sensorHit() {
    // Read the SmartDashboard input
    // coralDetected = SmartDashboard.getBoolean("Coral Detected Sim Input", false);
    return hit;
  }
}
