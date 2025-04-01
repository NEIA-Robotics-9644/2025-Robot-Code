package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private LoggedTunableNumber maxAmps = new LoggedTunableNumber("Climb/Max Amps", 40.0);
  private LoggedTunableNumber maxSpeedUp = new LoggedTunableNumber("Climb/Max Speed Up", 1.0);
  private LoggedTunableNumber maxSpeedDown = new LoggedTunableNumber("Climb/Max Speed Down", 1);
  private LoggedTunableNumber kP = new LoggedTunableNumber("Climb/P", 0.6);
  private LoggedTunableNumber kI = new LoggedTunableNumber("Climb/I", 0.0);
  private LoggedTunableNumber kD = new LoggedTunableNumber("Climb/D", 0.0);
  private LoggedTunableNumber kG = new LoggedTunableNumber("Climb/G", 0.0);
  private LoggedTunableNumber maxAngleRads = new LoggedTunableNumber("Climb/Max Angle Rads", 100.0);
  private LoggedTunableNumber minAngleRads = new LoggedTunableNumber("Climb/Min Angle Rads", 0.0);

  // MAKE THIS AN INTERFACE WHEN WE HAVE TIME
  private final Servo climbLockMotor = new Servo(9);

  private final Servo intakeReleaseMotor = new Servo(8);

  private final ClimberIO motorIO;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private PIDController pid = new PIDController(kP.get(), kI.get(), kD.get());

  public Climber(ClimberIO io) {
    this.motorIO = io;
    intakeReleaseMotor.set(0);
    climbLockMotor.set(0);
  }

  public void periodic() {
    motorIO.updateInputs(inputs);
    motorIO.periodic();
    Logger.recordOutput(
        "Climb/CurrentCommand",
        getCurrentCommand() != null ? getCurrentCommand().getName() : "None");
    Logger.processInputs("Climb", inputs);


    motorIO.setMaxAmps((int) maxAmps.get());
  }

  public void releaseIntake() {
    intakeReleaseMotor.set(1);
  }

  @AutoLogOutput(key = "Climb/IntakeReleased")
  public boolean intakeReleased() {
    return intakeReleaseMotor.getAngle() > 0.5;
  }

  public void lockClimb() {
    climbLockMotor.set(1);
  }

  @AutoLogOutput(key = "Climb/ClimbLocked")
  public boolean climbLocked() {
    return climbLockMotor.getAngle() > 0.5;
  }

  public void setVelocity(double velocity) {
    motorIO.setVelocity(velocity);
  }

  @AutoLogOutput(key = "Climb/CurrentAngleRads")
  public double getPositionRads() {
    return motorIO.getPositionRads();
  }

  public void goToAngle(double angleRads) {
    Logger.recordOutput("Climb/TargetDegreesFromVertical", angleRads);

    pid.setPID(kP.get(), kI.get(), kD.get());

    var setpoint = MathUtil.clamp(angleRads, minAngleRads.get(), maxAngleRads.get());

    double output = pid.calculate(getPositionRads(), setpoint + kG.get());
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


  public Command positionControl(DoubleSupplier positionChangeRadPerSec) {
    return Commands.run(
            () -> {
              var setpoint = getPositionRads() + (positionChangeRadPerSec.getAsDouble() * 0.02);
              goToAngle(setpoint);
            },
            this)
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        .withName("Climb Position Control");
  }

}
