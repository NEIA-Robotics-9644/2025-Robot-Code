package frc.robot;

public class ControllerState {

  public class ExtenderSetpoint {
    public double height;
    public double angle;

    public ExtenderSetpoint(double height, double angle) {
      this.height = height;
      this.angle = angle;
    }
  }

  public ExtenderSetpoint INTAKE = new ExtenderSetpoint(0, 0);
  public ExtenderSetpoint L1 = new ExtenderSetpoint(0.1, 0);
  public ExtenderSetpoint L2 = new ExtenderSetpoint(0.2, 15);
  public ExtenderSetpoint L3 = new ExtenderSetpoint(0.3, 15);
  public ExtenderSetpoint L4 = new ExtenderSetpoint(0.95, 30);

  private ExtenderSetpoint currentSetpoint = INTAKE;

  public ControllerState() {}

  public ExtenderSetpoint getCurrentSetpoint() {
    return currentSetpoint;
  }

  public void setCurrentSetpoint(ExtenderSetpoint currentSetpoint) {
    this.currentSetpoint = currentSetpoint;
  }
}
