package frc.robot.subsystems.extender.sensor;

import edu.wpi.first.wpilibj.DigitalInput;

public class LimitSwitchSensorIORoboRio implements LimitSwitchSensorIO {
  // Initializes a DigitalInput on DIO 9
  private DigitalInput input;

  public LimitSwitchSensorIORoboRio(int rioInputPin) {
    input = new DigitalInput(rioInputPin);
  }

  @Override
  public boolean sensorHit() {
    // Returns the value of the DigitalInput
    return input.get();
  }

  @Override
  public void updateInputs(LimitSwitchSensorIOInputs inputs) {
    inputs.sensorHit = sensorHit();
  }
}
