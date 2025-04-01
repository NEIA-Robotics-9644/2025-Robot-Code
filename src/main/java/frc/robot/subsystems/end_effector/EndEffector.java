package frc.robot.subsystems.end_effector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {

  private LoggedTunableNumber maxAmps = new LoggedTunableNumber("EndEffectorWheels/Max Amps", 40.0);

  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  public EndEffector(FlywheelIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    io.periodic();
    Logger.recordOutput(
        "EndEffectorWheels/CurrentCommand",
        getCurrentCommand() != null ? getCurrentCommand().getName() : "None");
    Logger.processInputs("EndEffector", inputs);

    io.setMaxAmps((int) maxAmps.get());
  }

  public void setVelocity(double velocity) {
    io.setVelocity(velocity);
  }

  public void zeroMotor() {
    io.zeroEncoder();
  }

  @AutoLogOutput(key = "EndEffectorWheels/CurrentAngleRads")
  public double getPositionRads() {
    return io.getPositionRads();
  }

  public Command manualControl(DoubleSupplier speed) {
    return Commands.run(
            () -> {
              setVelocity(speed.getAsDouble());
            },
            this)
        .withName("Flywheel Manual Control")
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }
}
