package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;

public class ControllerState {

  public class ExtenderSetpoint {
    public double height;
    public double angle;

    public ExtenderSetpoint(double heightNormalized, double angleRads) {
      this.height = heightNormalized;
      this.angle = angleRads;
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

  public ExtenderSetpoint INTAKE = new ExtenderSetpoint(0, 0);
  public ExtenderSetpoint L1 = new ExtenderSetpoint(2.449 / 21, 0.47);
  public ExtenderSetpoint L2 = new ExtenderSetpoint(0.2851 - 0.04, 0.45);
  public ExtenderSetpoint L3 = new ExtenderSetpoint(0.55 * 0.8571428571 + 0.04, 0.48);
  public ExtenderSetpoint L4 =
      new ExtenderSetpoint(0.8571428571 + 0.06 - 0.015, 0.81 - 0.07 + 0.07);
  public ExtenderSetpoint LowDealgify = new ExtenderSetpoint(0.225, 0.84);
  public ExtenderSetpoint HighDealgify = new ExtenderSetpoint(0.4, 0.84);

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
          System.out.println("Going to L4");
        });
  }

  public Command runManualSetpoint(DoubleSupplier heightChange, DoubleSupplier angleChange) {
    return Commands.run(
        () -> {
          this.setCurrentSetpoint(
              new ExtenderSetpoint(
                  MathUtil.clamp(
                      this.getCurrentSetpoint().height + heightChange.getAsDouble(), 0, 1),
                  MathUtil.clamp(
                      this.getCurrentSetpoint().angle + angleChange.getAsDouble(), 0, 1)));
        });
  }
}
