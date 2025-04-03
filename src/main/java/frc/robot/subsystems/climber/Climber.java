package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private LoggedTunableNumber maxAmps = new LoggedTunableNumber("Climb/Max Amps", 40.0);
  private LoggedTunableNumber maxSpeedUp = new LoggedTunableNumber("Climb/Max Speed Up", 1.0);
  private LoggedTunableNumber maxSpeedDown = new LoggedTunableNumber("Climb/Max Speed Down", 1);
  private LoggedTunableNumber maxAngleRads = new LoggedTunableNumber("Climb/Max Angle Rads", 100.0);
  private LoggedTunableNumber minAngleRads = new LoggedTunableNumber("Climb/Min Angle Rads", 0.0);

  // MAKE THIS AN INTERFACE WHEN WE HAVE TIME
  private Servo climbLockMotor;

  private Servo intakeReleaseMotor;

  private final ClimberIO motorIO;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  public Climber(ClimberIO io, int intakeReleaseChannel, int climbLockChannel) {
    this.motorIO = io;
    this.intakeReleaseMotor = new Servo(intakeReleaseChannel);
    this.climbLockMotor = new Servo(climbLockChannel);
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

    if (this.intakeReleased()) {
      Logger.recordOutput("Climb/IntakeState", "Intake Released");
    } else {
      Logger.recordOutput("Climb/IntakeState", "Intake Locked");
    }

    motorIO.setMaxAmps((int) maxAmps.get());
  }

  public void toggleIntake() {
    if (intakeReleased()) {
      intakeReleaseMotor.set(0); // retract
    } else {
      intakeReleaseMotor.set(0.5); // extend
    }
  }

  @AutoLogOutput(key = "Climb/IntakeReleased")
  public boolean intakeReleased() {
    return this.intakeReleaseMotor.getAngle() > 0.25;
  }

  public void toggleClimbLock() {
    if (climbLocked()) {
      climbLockMotor.setAngle(-30); // unlock
    } else {
      climbLockMotor.set(1); // lock
    }
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

  public Command moveWithVelocity(DoubleSupplier velocity) {
    return Commands.run(
            () -> {
              motorIO.setVelocity(velocity.getAsDouble());
            },
            this)
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        .withName("Climb Position Control");
  }
}
