package frc.robot.subsystems.climb;

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

public class Climb extends SubsystemBase {

  private LoggedTunableNumber maxAmps = new LoggedTunableNumber("Climb/Max Amps", 40.0);
  private LoggedTunableNumber maxSpeedUp = new LoggedTunableNumber("Climb/Max Speed Up", 1.0);
  private LoggedTunableNumber maxSpeedDown = new LoggedTunableNumber("Climb/Max Speed Down", 1);
  private LoggedTunableNumber kP = new LoggedTunableNumber("Climb/P", 0.6);
  private LoggedTunableNumber kI = new LoggedTunableNumber("Climb/I", 0.0);
  private LoggedTunableNumber kD = new LoggedTunableNumber("Climb/D", 0.0);
  private LoggedTunableNumber kG = new LoggedTunableNumber("Climb/G", -0.024);

  private LoggedTunableNumber degreeAngleWhenUp =
      new LoggedTunableNumber("Climb/DegreeAngleWhenUp", 10.0);

  private LoggedTunableNumber angleOffsetRads =
      new LoggedTunableNumber("Climb/AngleOffsetRads", 2.34);

  private LoggedTunableNumber maxAngleRads = new LoggedTunableNumber("Climb/MaxAngleRads", 1.9);

  private LoggedTunableNumber motorScaleFactor =
      new LoggedTunableNumber("Climb/MotorScaleFactor", 1.0);

  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  private PIDController pid = new PIDController(kP.get(), kI.get(), kD.get());

  public Climb(ClimbIO io) {
    this.io = io;
  }

  public double degreesFromVerticalToRadians(double degrees) {
    return Math.toRadians(degrees - degreeAngleWhenUp.get());
  }

  public double radiansToDegreesFromVertical(double radians) {
    return Math.toDegrees(radians) + degreeAngleWhenUp.get();
  }

  public double getMaxDegreesFromVertical() {
    return radiansToDegreesFromVertical(maxAngleRads.get());
  }

  public double getMinDegreesFromVertical() {
    return degreeAngleWhenUp.get();
  }

  public void periodic() {
    io.updateInputs(inputs);
    io.periodic();
    Logger.recordOutput(
        "Climb/CurrentCommand",
        getCurrentCommand() != null ? getCurrentCommand().getName() : "None");
    Logger.processInputs("Climb", inputs);

    io.setMaxAmps((int) maxAmps.get());
    io.setInverted(motorScaleFactor.get() < 0.0);
    io.setEncoderOffset(angleOffsetRads.get());
  }

  public void setVelocity(double velocity) {
    io.setVelocity(velocity);
  }

  @AutoLogOutput(key = "Climb/CurrentAngleRads")
  public double getPositionRads() {
    return io.getPositionRads();
  }

  @AutoLogOutput(key = "Climb/CurrentAngleDegreesFromVertical")
  public double getPositionDegreesFromVertical() {
    return radiansToDegreesFromVertical(getPositionRads());
  }

  public void goToAngle(double degreesFromVertical) {
    Logger.recordOutput("Climb/TargetDegreesFromVertical", degreesFromVertical);

    pid.setPID(kP.get(), kI.get(), kD.get());

    var setpoint =
        MathUtil.clamp(degreesFromVerticalToRadians(degreesFromVertical), 0, maxAngleRads.get());

    double output = pid.calculate(getPositionRads(), setpoint) + kG.get();
    output = MathUtil.clamp(output, -maxSpeedUp.get(), maxSpeedDown.get());

    setVelocity(output);
  }

  public Command goToAngle(DoubleSupplier degreesFromVertical) {
    return Commands.run(
            () -> {
              var setpoint = degreesFromVertical.getAsDouble();
              goToAngle(setpoint);
            },
            this)
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        .withName("Climb Go To Angle");
  }

  public Command ClimbConstraintsCommand(DoubleSupplier targetDegrees) {

    return Commands.run(
            () -> {
              Logger.recordOutput("Climb/MinAngle", 0);
              Logger.recordOutput("Climb/MaxAngle", 360);
              Logger.recordOutput("Climb/TargetAngle", targetDegrees.getAsDouble());

              var climbAngle = targetDegrees.getAsDouble();
              var clampedAngle = MathUtil.clamp(climbAngle, 0, 360);
              System.out.println("Clamped angle: " + clampedAngle);
              this.goToAngle(clampedAngle);
            },
            this)
        .withName("Climb Go To Angle With Angle Constraints");
  }
}
