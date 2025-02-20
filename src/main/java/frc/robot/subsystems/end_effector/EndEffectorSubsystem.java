package frc.robot.subsystems.end_effector;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class EndEffectorSubsystem extends SubsystemBase {

  private LoggedTunableNumber maxAmps = new LoggedTunableNumber("Flywheel/Max Amps", 40.0);
  private LoggedTunableNumber maxSpeedUp = new LoggedTunableNumber("Flywheel/Max Speed Up", 1.0);
  private LoggedTunableNumber maxSpeedDown = new LoggedTunableNumber("Flywheel/Max Speed Down", 1);
  private LoggedTunableNumber kP = new LoggedTunableNumber("Flywheel/P", 0.5);
  private LoggedTunableNumber kI = new LoggedTunableNumber("Flywheel/I", 0.0);
  private LoggedTunableNumber kD = new LoggedTunableNumber("Flywheel/D", 0.0);

  private LoggedTunableNumber kAngle = new LoggedTunableNumber("Flywheel/AngleGain", 0.1);

  private LoggedTunableNumber angleOffsetRads =
      new LoggedTunableNumber("Flywheel/AngleOffsetRads", 0.0);

  private LoggedTunableNumber motorScaleFactor =
      new LoggedTunableNumber("Flywheel/MotorScaleFactor", 1.0);

  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  private PIDController pid = new PIDController(kP.get(), kI.get(), kD.get());

  public EndEffectorSubsystem(FlywheelIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    io.periodic();
    Logger.recordOutput(
        "Flywheel/CurrentCommand",
        getCurrentCommand() != null ? getCurrentCommand().getName() : "None");
    Logger.processInputs("EndEffector", inputs);

    io.setMaxAmps((int) maxAmps.get());
    io.setInverted(motorScaleFactor.get() < 0.0);
    io.setEncoderOffset(angleOffsetRads.get());
  }

  public void setVelocity(double velocity) {
    io.setVelocity(velocity);
  }

  public void zeroMotor() {
    io.zeroEncoder();
  }

  @AutoLogOutput(key = "Flywheel/CurrentAngleRads")
  public double getPositionRads() {
    return io.getPositionRads();
  }

  public Command goToAngle(DoubleSupplier angleRad) {
    return Commands.run(
            () -> {
              Logger.recordOutput("Flywheel/TargetAngleRads", angleRad.getAsDouble());

              pid.setPID(kP.get(), kI.get(), kD.get());

              // Assuming Flywheel is zero when vertical, sin will be 0 when vertical and 1 when
              // horizontal (which is where we need most strength to hold the Flywheel up)
              double output =
                  pid.calculate(getPositionRads(), angleRad.getAsDouble())
                      + Math.sin(angleRad.getAsDouble()) * kAngle.get();
              output = MathUtil.clamp(output, -maxSpeedUp.get(), maxSpeedDown.get());

              setVelocity(output);
            },
            this)
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        .withName("Flywheel Go To Angle");
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
