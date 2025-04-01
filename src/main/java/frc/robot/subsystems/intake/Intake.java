package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final IntakeWheelIO wheel;

  private final IntakeWheelIOInputsAutoLogged inputs = new IntakeWheelIOInputsAutoLogged();

  public Intake(IntakeWheelIO wheel) {
    this.wheel = wheel;
  }

  public void periodic() {
    wheel.updateInputs(inputs);
    wheel.periodic();
    Logger.processInputs("IntakeWheel", inputs);
  }

  public void setVelocity(double velocity) {
    wheel.setVelocity(velocity);
  }
}
