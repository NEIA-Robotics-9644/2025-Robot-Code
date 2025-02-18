package frc.robot.subsystems.pivot;

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

public class Pivot extends SubsystemBase {

  private LoggedTunableNumber maxAmps = new LoggedTunableNumber("Pivot/Max Amps", 40.0);
  private LoggedTunableNumber maxSpeedUp = new LoggedTunableNumber("Pivot/Max Speed Up", 1.0);
  private LoggedTunableNumber maxSpeedDown = new LoggedTunableNumber("Pivot/Max Speed Down", 1);
  private LoggedTunableNumber kP = new LoggedTunableNumber("Pivot/P", 0.5);
  private LoggedTunableNumber kI = new LoggedTunableNumber("Pivot/I", 0.0);
  private LoggedTunableNumber kD = new LoggedTunableNumber("Pivot/D", 0.0);

  private LoggedTunableNumber kAngle = new LoggedTunableNumber("Pivot/AngleGain", 0.1);

  private LoggedTunableNumber angleOffsetRads =
      new LoggedTunableNumber("Pivot/AngleOffsetRads", 0.0);

  private LoggedTunableNumber motorScaleFactor =
      new LoggedTunableNumber("Pivot/MotorScaleFactor", 1.0);

  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  private PIDController pid = new PIDController(kP.get(), kI.get(), kD.get());

  public Pivot(PivotIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);

    io.setMaxAmps((int) maxAmps.get());
    io.setInverted(motorScaleFactor.get() < 0.0);
    io.setEncoderOffset(angleOffsetRads.get());
  }

  public void setVelocity(double velocity) {
    io.setVelocity(velocity);
  }

  @AutoLogOutput(key = "Pivot/CurrentAngleRads")
  public double getPositionRads() {
    return io.getPositionRads();
  }

  public Command goToAngle(DoubleSupplier angleRad) {
    return Commands.run(
            () -> {
              Logger.recordOutput("Pivot/TargetAngle", angleRad.getAsDouble());

              pid.setPID(kP.get(), kI.get(), kD.get());

              // Assuming pivot is zero when vertical, sin will be 0 when vertical and 1 when
              // horizontal (which is where we need most strength to hold the pivot up)
              double output =
                  pid.calculate(
                      getPositionRads(),
                      angleRad.getAsDouble() + Math.sin(angleRad.getAsDouble()) * kAngle.get());
              output = MathUtil.clamp(output, -maxSpeedUp.get(), maxSpeedDown.get());

              setVelocity(output);
            },
            this)
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        .withName("Pivot Go To Angle");
  }

  public Command manualControl(DoubleSupplier speed) {
    return Commands.run(
            () -> {
              setVelocity(speed.getAsDouble());
            },
            this)
        .withName("Pivot Manual Control")
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }
}
