package frc.robot.subsystems.end_effector_wheels;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class EndEffectorWheels extends SubsystemBase {

  private final FlywheelIO wheels;

  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  public EndEffectorWheels(FlywheelIO wheels) {
    this.wheels = wheels;
  }

  public void periodic() {
    wheels.updateInputs(inputs);
    Logger.processInputs("EndEffectorWheels", inputs);
  }

  public void setVelocity(double velocity) {
    wheels.runVelocity(velocity);
  }
}
