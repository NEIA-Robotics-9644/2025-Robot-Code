package frc.robot.subsystems.extender.sensor;

import edu.wpi.first.wpilibj.DigitalInput;

public class LimitSwitchSensorIORoboRio implements LimitSwitchSensorIO {
  // Initializes a DigitalInput on DIO 9
  private DigitalInput input = new DigitalInput(9);

  public LimitSwitchSensorIORoboRio() {}

  @Override
  public boolean sensorHit() {
    // Returns the value of the DigitalInput
    return input.get();
  }
}
