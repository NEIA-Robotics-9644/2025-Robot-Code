package frc.robot.subsystems.intake.sensor;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

public class CoralSensorIORoboRio implements CoralSensorIO {
  // Initializes a DigitalInput on DIO 5
  private DigitalInput input = new DigitalInput(5);

  private PowerDistribution pdh;

  public CoralSensorIORoboRio() {
    this.pdh = new PowerDistribution(1, ModuleType.kRev);
  }

  @Override
  public boolean coralDetected() {
    // Returns the value of the DigitalInput

    return input.get();
  }

  @Override
  public void setDisplayLight(boolean on) {
    pdh.setSwitchableChannel(on);
  }
}
