package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Pivot;
import java.util.function.DoubleSupplier;

public class ControllerState extends SubsystemBase {

  public class ExtenderSetpoint {
    public double inchesFromGround;
    public double degreesFromVertical;

    public ExtenderSetpoint(double inchesFromGround, double degreesFromVertical) {
      this.inchesFromGround = inchesFromGround;
      this.degreesFromVertical = degreesFromVertical;
    }
  }

  public class DriveSpeed {
    public double translationScale;
    public double rotationScale;

    public DriveSpeed(double translationScale, double rotationScale) {
      this.translationScale = translationScale;
      this.rotationScale = rotationScale;
    }
  }

  public ExtenderSetpoint INTAKE = new ExtenderSetpoint(12.5, 10);
  public ExtenderSetpoint L1 = new ExtenderSetpoint(20 - 1, 10);
  public ExtenderSetpoint L2 = new ExtenderSetpoint(26, 30);
  public ExtenderSetpoint L3 = new ExtenderSetpoint(44, 30);
  public ExtenderSetpoint L4 = new ExtenderSetpoint(75, 70);
  public ExtenderSetpoint LowDealgify = new ExtenderSetpoint(25, 70);
  public ExtenderSetpoint HighDealgify = new ExtenderSetpoint(45, 70);

  private ExtenderSetpoint currentSetpoint = INTAKE;

  public DriveSpeed[] driveSpeeds = {
    new DriveSpeed(0.5, 0.5), new DriveSpeed(0.75, 0.75), new DriveSpeed(1.0, 1.0)
  };

  public int currentDriveSpeedIndex = 0;

  public ControllerState() {}

  @Override
  public void periodic() {}

  public ExtenderSetpoint getCurrentSetpoint() {
    return currentSetpoint;
  }

  public void setCurrentSetpoint(ExtenderSetpoint currentSetpoint) {
    this.currentSetpoint = currentSetpoint;
  }

  public DriveSpeed getCurrentDriveSpeed() {
    return driveSpeeds[currentDriveSpeedIndex];
  }

  public void increaseDriveSpeedIndex() {
    if (currentDriveSpeedIndex < driveSpeeds.length - 1) {
      currentDriveSpeedIndex++;
    }
  }

  public void decreaseDriveSpeedIndex() {
    if (currentDriveSpeedIndex > 0) {
      currentDriveSpeedIndex--;
    }
  }

  public Command setSetpoint(ExtenderSetpoint setpoint) {
    return Commands.runOnce(
        () -> {
          this.setCurrentSetpoint(setpoint);
        });
  }

  public Command runManualSetpoint(
      Elevator elevator, Pivot pivot, DoubleSupplier heightChange, DoubleSupplier angleChange) {
    return Commands.run(
            () -> {
              this.setCurrentSetpoint(
                  new ExtenderSetpoint(
                      MathUtil.clamp(
                          this.getCurrentSetpoint().inchesFromGround + heightChange.getAsDouble(),
                          elevator.getMinInchesFromGround(),
                          elevator.getMaxInchesFromGround()),
                      MathUtil.clamp(
                          this.getCurrentSetpoint().degreesFromVertical + angleChange.getAsDouble(),
                          pivot.getMinDegreesFromVertical(),
                          pivot.getMaxDegreesFromVertical())));

              if (elevator.limitSwitchTripped()) {
                elevator.zeroEncoders();
                System.out.println("In manual mode, zeroed encoders");
              }
            })
        .alongWith(elevator.goToHeight(() -> currentSetpoint.inchesFromGround))
        .alongWith(pivot.goToAngle(() -> currentSetpoint.degreesFromVertical))
        .withName("Manual Extender Control");
  }
}
