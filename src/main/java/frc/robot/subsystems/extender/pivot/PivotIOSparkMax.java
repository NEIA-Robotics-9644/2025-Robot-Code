package frc.robot.subsystems.extender.pivot;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class PivotIOSparkMax implements PivotIO {

  private final double bottomLimitDeg = 0.0;
  private final double topLimitDeg = 1000;

  private final SparkMax motor;

  private SparkMaxConfig config = new SparkMaxConfig();

  private final double reduction;

  private double normalizedVelocity = 0.0;

  private boolean manualControl = false;

  private final double encoderOffsetDeg = 0.0;

  private final double physicalMaxSpeed = 100;

  private double startingOffsetDeg = 0;

  private final double maxSpeedDegPerSec = 25.0;

  private final boolean encoderReversed = false;

  private double encoderReadingRotationsToAngleDeg = 360.0 / 80.0;

  final int maxCurrentA = 20;

  private PIDController pid = new PIDController(2, 0.0, 0.0);
  private final double kG = 5;

  private final Double position;
  private final Double velocity;
  private final Double appliedVoltage;
  private final Double outputCurrent;
  private final Double tempCelsius;

  public PivotIOSparkMax(int id, double reduction) {
    this.motor = new SparkMax(id, SparkMax.MotorType.kBrushless);

    config.idleMode(SparkMaxConfig.IdleMode.kBrake);

    config.smartCurrentLimit(maxCurrentA);
    config.openLoopRampRate(0.5);

    this.motor.configure(
        config,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters);

    this.reduction = reduction;

    position = motor.getAbsoluteEncoder().getPosition();
    velocity = motor.getAbsoluteEncoder().getVelocity() / 2;
    appliedVoltage = motor.getBusVoltage();
    outputCurrent = motor.getOutputCurrent();
    tempCelsius = motor.getMotorTemperature();
  }

  @Override
  public void setAngleSetpoint(String height) {
    // Clamp the setpoint between the higher and lower limits
    double setpointClamped =
        Math.max(bottomLimitDeg, Math.min(topLimitDeg, Constants.extenderAngles.get(height)));
    pid.setSetpoint(setpointClamped);
    this.manualControl = false;
  }

  @Override
  public void setManualVelocity(double normalizedVelocity) {

    this.normalizedVelocity = Math.max(-1.0, Math.min(1.0, normalizedVelocity));
    this.manualControl = true;
  }

  private double maxSpeedRPM = 5000.0;

  @Override
  public double getVelocityPercent() {
    return motor.getEncoder().getVelocity() / maxSpeedRPM;
  }

  @Override
  public void periodic() {
    if (!manualControl) {

      // Calculate the pid
      double pidOutput = pid.calculate(getAngleDeg());

      double gravityComp = kG * Math.cos(Math.toRadians(getAngleDeg()));

      double output = pidOutput + gravityComp;

      // Clamp so it is under the max speed
      if (output > maxSpeedDegPerSec) {
        output = maxSpeedDegPerSec;
      } else if (output < -maxSpeedDegPerSec) {
        output = -maxSpeedDegPerSec;
      }

      this.motor.set(output * physicalMaxSpeed);
    } else {

      this.motor.set(normalizedVelocity * physicalMaxSpeed);
    }
  }

  @Override
  public double getAngleDeg() {

    double angle =
        motor.getEncoder().getPosition()
            * encoderReadingRotationsToAngleDeg
            * (encoderReversed ? -1 : 1);
    return angle + encoderOffsetDeg + startingOffsetDeg;
  }

  @Override
  public void setBrakeMode(boolean brake) {
    config.idleMode(brake ? SparkMaxConfig.IdleMode.kBrake : SparkMaxConfig.IdleMode.kCoast);
    this.motor.configure(
        config,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.positionRads = Units.rotationsToRadians(position) / reduction;
    inputs.velocityRadsPerSec = Units.rotationsToRadians(velocity * (2 * Math.PI / 60)) / reduction;
    inputs.appliedVoltage = appliedVoltage;
    inputs.outputCurrentAmps = outputCurrent;
    inputs.tempCelsius = tempCelsius;
  }
}
