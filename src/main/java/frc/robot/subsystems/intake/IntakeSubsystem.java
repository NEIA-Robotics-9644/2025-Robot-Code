package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.sensor.CoralSensorIO;
import frc.robot.subsystems.intake.wheel.IntakeWheelIO;
import frc.robot.subsystems.intake.wheel.IntakeWheelIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

  private final IntakeWheelIO wheel;

  private final CoralSensorIO coral;

  private final IntakeWheelIOInputsAutoLogged inputs = new IntakeWheelIOInputsAutoLogged();

  public IntakeSubsystem(IntakeWheelIO wheel, CoralSensorIO coral) {
    this.wheel = wheel;
    this.coral = coral;
  }

  public void periodic() {
    wheel.updateInputs(inputs);
    wheel.periodic();
    Logger.processInputs("IntakeWheel", inputs);
  }

  public void setVelocity(double velocity) {
    wheel.setVelocity(velocity);
  }

  public boolean sensorState() {
    return coral.coralDetected();
  }
}
