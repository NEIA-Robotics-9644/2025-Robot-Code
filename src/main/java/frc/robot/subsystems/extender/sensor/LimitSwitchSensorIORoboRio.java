package frc.robot.subsystems.extender.sensor;

import edu.wpi.first.wpilibj.DigitalInput;

public class LimitSwitchSensorIORoboRio implements LimitSwitchSensorIO {
  // Initializes a DigitalInput on DIO 9
  private DigitalInput input;
  private boolean defaultOn = false;

  public LimitSwitchSensorIORoboRio(int rioInputPin, boolean defaultOn) {
    input = new DigitalInput(rioInputPin);
    this.defaultOn = defaultOn;
  }

  @Override
  public boolean sensorHit() {
    // Returns the value of the DigitalInput
    if (defaultOn) {
      return !input.get();
    } else {
      return input.get();
    }
  }

  @Override
  public void updateInputs(LimitSwitchSensorIOInputs inputs) {
    inputs.sensorHit = sensorHit();
  }
}
