package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ExtenderConstraints;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {

  private LoggedTunableNumber maxAmps = new LoggedTunableNumber("Pivot/Max Amps", 40.0);
  private LoggedTunableNumber maxSpeedUp = new LoggedTunableNumber("Pivot/Max Speed Up", 1.0);
  private LoggedTunableNumber maxSpeedDown = new LoggedTunableNumber("Pivot/Max Speed Down", 1);
  private LoggedTunableNumber kP = new LoggedTunableNumber("Pivot/P", 0.6);
  private LoggedTunableNumber kI = new LoggedTunableNumber("Pivot/I", 0.0);
  private LoggedTunableNumber kD = new LoggedTunableNumber("Pivot/D", 0.0);
  private LoggedTunableNumber kG = new LoggedTunableNumber("Pivot/G", -0.024);

  private LoggedTunableNumber degreeAngleWhenUp =
      new LoggedTunableNumber("Pivot/DegreeAngleWhenUp", 10.0);

  private LoggedTunableNumber angleOffsetRads =
      new LoggedTunableNumber("Pivot/AngleOffsetRads", 2.34);

  private LoggedTunableNumber maxAngleRads = new LoggedTunableNumber("Pivot/MaxAngleRads", 1.9);

  private LoggedTunableNumber motorScaleFactor =
      new LoggedTunableNumber("Pivot/MotorScaleFactor", 1.0);

  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  private PIDController pid = new PIDController(kP.get(), kI.get(), kD.get());

  public Pivot(PivotIO io) {
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
        "Pivot/CurrentCommand",
        getCurrentCommand() != null ? getCurrentCommand().getName() : "None");
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

  @AutoLogOutput(key = "Pivot/CurrentAngleDegreesFromVertical")
  public double getPositionDegreesFromVertical() {
    return radiansToDegreesFromVertical(getPositionRads());
  }

  public void goToAngle(double degreesFromVertical) {
    Logger.recordOutput("Pivot/TargetDegreesFromVertical", degreesFromVertical);

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
        .withName("Pivot Go To Angle");
  }

  public Command pivotConstraintsCommand(
      DoubleSupplier inchesFromGround,
      ExtenderConstraints extenderConstraints,
      DoubleSupplier targetDegreesFromVertical) {

    return Commands.run(
            () -> {
              var constraint =
                  extenderConstraints.getAngleConstraint(inchesFromGround.getAsDouble());

              Logger.recordOutput("Pivot/MinAngle", constraint.minDegreesFromVertical());
              Logger.recordOutput("Pivot/MaxAngle", constraint.maxDegreesFromVertical());
              Logger.recordOutput("Pivot/TargetAngle", targetDegreesFromVertical.getAsDouble());

              var pivotAngle = targetDegreesFromVertical.getAsDouble();
              var clampedAngle =
                  MathUtil.clamp(
                      pivotAngle,
                      constraint.minDegreesFromVertical(),
                      constraint.maxDegreesFromVertical());
              this.goToAngle(clampedAngle);
            },
            this)
        .withName("Pivot Go To Angle With Angle Constraints");
  }
}
