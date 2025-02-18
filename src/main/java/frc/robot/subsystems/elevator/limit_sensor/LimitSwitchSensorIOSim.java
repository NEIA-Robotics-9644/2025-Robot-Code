package frc.robot.subsystems.elevator.limit_sensor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimitSwitchSensorIOSim implements LimitSwitchSensorIO {

  // Simulate a simple sensor

  public LimitSwitchSensorIOSim() {
    SmartDashboard.putBoolean("Sim Limit Switch Tripped", false);
  }

  @Override
  public boolean sensorHit() {
    return SmartDashboard.getBoolean("Sim Limit Switch Tripped", false);
  }
}
