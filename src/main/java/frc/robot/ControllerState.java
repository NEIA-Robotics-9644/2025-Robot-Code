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

  public ExtenderSetpoint INTAKE = new ExtenderSetpoint(0, 0);
  public ExtenderSetpoint L1 = new ExtenderSetpoint(0.1, 0);
  public ExtenderSetpoint L2 = new ExtenderSetpoint(0.2, 0.5);
  public ExtenderSetpoint L3 = new ExtenderSetpoint(0.5, 0.5);
  public ExtenderSetpoint L4 = new ExtenderSetpoint(0.95, 1);

  private ExtenderSetpoint currentSetpoint = INTAKE;

  public ControllerState() {}

  public ExtenderSetpoint getCurrentSetpoint() {
    return currentSetpoint;
  }

  public void setCurrentSetpoint(ExtenderSetpoint currentSetpoint) {
    this.currentSetpoint = currentSetpoint;
  }
}
