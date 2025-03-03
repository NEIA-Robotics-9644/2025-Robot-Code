package frc.robot;

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
  public ExtenderSetpoint L1 = new ExtenderSetpoint(0.17, 0);
  public ExtenderSetpoint L2 = new ExtenderSetpoint(0.286, 0.48);
  public ExtenderSetpoint L3 = new ExtenderSetpoint(0.55, 0.48);
  public ExtenderSetpoint L4 = new ExtenderSetpoint(1, 0.81);
  public ExtenderSetpoint LowDealgify = new ExtenderSetpoint(0.286, 0.52);
  public ExtenderSetpoint HighDealgify = new ExtenderSetpoint(0.52, 0.52);

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
}
