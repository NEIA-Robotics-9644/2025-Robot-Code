package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Pivot;
import java.util.function.DoubleSupplier;

public class ControllerState extends SubsystemBase {

  NetworkTableEntry useSetpointQueueingEntry;

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
  public ExtenderSetpoint L1 = new ExtenderSetpoint(20, 10);
  public ExtenderSetpoint L2 = new ExtenderSetpoint(26, 30);
  public ExtenderSetpoint L3 = new ExtenderSetpoint(44, 30);
  public ExtenderSetpoint L4 = new ExtenderSetpoint(75, 70);
  public ExtenderSetpoint LowDealgify = new ExtenderSetpoint(24, 0.84);
  public ExtenderSetpoint HighDealgify = new ExtenderSetpoint(25, 0.84);

  private ExtenderSetpoint currentSetpoint = INTAKE;

  private ExtenderSetpoint queuedSetpoint = currentSetpoint;

  public DriveSpeed[] driveSpeeds = {
    new DriveSpeed(0.5, 0.5), new DriveSpeed(0.75, 0.75), new DriveSpeed(1.0, 1.0)
  };

  public int currentDriveSpeedIndex = 0;

  public ControllerState() {
    // Initialize the useSetpointQueueing entry

    useSetpointQueueingEntry =
        NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getEntry("Use Setpoint Queueing");
    useSetpointQueueingEntry.setBoolean(true); // Default to not using queueing
  }

  @Override
  public void periodic() {
    // Update the useSetpointQueueing entry on the SmartDashboard
    if (!useSetpointQueueingEntry.getBoolean(false)) {
      currentSetpoint =
          queuedSetpoint; // If queueing is disabled, go immediately to the queued setpoint
    }
  }

  public ExtenderSetpoint getCurrentSetpoint() {
    return currentSetpoint;
  }

  public void setCurrentSetpoint(ExtenderSetpoint currentSetpoint) {
    this.currentSetpoint = currentSetpoint;
  }

  public void setQueuedSetpoint(ExtenderSetpoint queuedSetpoint) {
    this.queuedSetpoint = queuedSetpoint;
  }

  public void activateQueuedSetpoint() {
    this.currentSetpoint = this.queuedSetpoint;
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
