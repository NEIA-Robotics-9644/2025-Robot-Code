package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Pivot;
import java.util.function.DoubleSupplier;

public class ControllerState {

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
  public ExtenderSetpoint L1 = new ExtenderSetpoint(15, 10);
  public ExtenderSetpoint L2 = new ExtenderSetpoint(25, 30);
  public ExtenderSetpoint L3 = new ExtenderSetpoint(45, 30);
  public ExtenderSetpoint L4 = new ExtenderSetpoint(75, 70);
  public ExtenderSetpoint LowDealgify = new ExtenderSetpoint(24, 0.84);
  public ExtenderSetpoint HighDealgify = new ExtenderSetpoint(25, 0.84);

  private ExtenderSetpoint currentSetpoint = INTAKE;

  public DriveSpeed[] driveSpeeds = {
    new DriveSpeed(0.5, 0.5), new DriveSpeed(0.75, 0.75), new DriveSpeed(1.0, 1.0)
  };

  public int currentDriveSpeedIndex = 0;

  public ControllerState() {}

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
        });
  }
}
