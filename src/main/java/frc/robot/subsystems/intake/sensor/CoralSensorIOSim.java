package frc.robot.subsystems.intake.sensor;

// import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class CoralSensorIOSim implements CoralSensorIO {

  private boolean coralDetected = false;

  // Simulate a simple sensor
  // Use an input from SmartDashboard to simulate the sensor

  public CoralSensorIOSim() {
    // Set up the SmartDashboard input
    // SmartDashboard.putBoolean("Coral Detected Sim Input", false);
  }

  public boolean coralDetected() {
    // Read the SmartDashboard input
    // coralDetected = SmartDashboard.getBoolean("Coral Detected Sim Input", false);
    return coralDetected;
  }

  public void setDisplayLight(boolean on) {
    // SmartDashboard.putBoolean("Sim Coral Detected", on);
  }
}
