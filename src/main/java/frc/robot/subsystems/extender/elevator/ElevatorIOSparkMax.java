package frc.robot.subsystems.extender.elevator;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ElevatorIOSparkMax implements ElevatorIO {
  private final double bottomLimitDeg = 0.0;
  private final double topLimitDeg = 1000;

  private final double encoderOffsetDeg = 0.0;

  private double startingOffsetDeg = 0;

  private final boolean encoderReversed = false;

  private final double physicalMaxSpeed = 100;

  private final SparkMax leftMotor;
  private final SparkMax rightMotor;

  private final boolean leftReversed = false;
  private final boolean rightReversed = true;

  private double encoderReadingRotationsToAngleDeg = 360.0 / 80.0;

  private final double maxSpeedDegPerSec = 25.0;

  private boolean manualControl = false;

  private PIDController pid = new PIDController(2, 0.0, 0.0);
  private final double kG = 5;

  private final Double positionL;
  private final Double velocityL;
  private final Double appliedVoltageL;
  private final Double outputCurrentL;
  private final Double tempCelsiusL;

  private final Double positionR;
  private final Double velocityR;
  private final Double appliedVoltageR;
  private final Double outputCurrentR;
  private final Double tempCelsiusR;

  private final Double reductionL;
  private final Double reductionR;

  public ElevatorIOSparkMax() {
    throw new IllegalArgumentException("You must pass in valid hardware for a subsystem to work");
  }

  public ElevatorIOSparkMax(
      int leftMotorCanID, int rightMotorCanID, double reductionL, double reductionR) {
    this.leftMotor = new SparkMax(leftMotorCanID, SparkMax.MotorType.kBrushless);
    this.rightMotor = new SparkMax(rightMotorCanID, SparkMax.MotorType.kBrushless);

    SparkMaxConfig lConfig = new SparkMaxConfig();
    SparkMaxConfig rConfig = new SparkMaxConfig();

    lConfig.inverted(leftReversed);

    rConfig.follow(leftMotorCanID, leftReversed != rightReversed);
    lConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
    rConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);

    lConfig.smartCurrentLimit(10);
    rConfig.smartCurrentLimit(10);
    rConfig.openLoopRampRate(0.5);
    rConfig.openLoopRampRate(0.5);

    this.leftMotor.configure(
        lConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters);
    this.rightMotor.configure(
        rConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters);

    this.pid.setTolerance(0.1);

    this.reductionL = reductionL;
    this.reductionR = reductionR;

    positionL = leftMotor.getAbsoluteEncoder().getPosition();
    velocityL = leftMotor.getAbsoluteEncoder().getVelocity() / 2;
    appliedVoltageL = leftMotor.getBusVoltage();
    outputCurrentL = leftMotor.getOutputCurrent();
    tempCelsiusL = leftMotor.getMotorTemperature();

    positionR = rightMotor.getAbsoluteEncoder().getPosition();
    velocityR = rightMotor.getAbsoluteEncoder().getVelocity() / 2;
    appliedVoltageR = rightMotor.getBusVoltage();
    outputCurrentR = rightMotor.getOutputCurrent();
    tempCelsiusR = rightMotor.getMotorTemperature();
  }

  @Override
  public void setAngleSetpoint(String height) {
    // Clamp the setpoint between the higher and lower limits
    double setpointClamped =
        Math.max(bottomLimitDeg, Math.min(topLimitDeg, Constants.extenderAngles.get(height)));
    pid.setSetpoint(setpointClamped);
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

      this.leftMotor.set(output / physicalMaxSpeed);
      this.rightMotor.set(output / physicalMaxSpeed);
    }
  }

  @Override
  public double getVelocityPercent() {
    return (leftMotor.getEncoder().getVelocity()
            * (1 / 80.0)
            * (encoderReversed ? -1 : 1)
            * 360.0
            * (1 / 60.0))
        / maxSpeedDegPerSec;
  }

  @Override
  public double getAngleDeg() {

    double angle =
        leftMotor.getEncoder().getPosition()
            * encoderReadingRotationsToAngleDeg
            * (encoderReversed ? -1 : 1);
    return angle + encoderOffsetDeg + startingOffsetDeg;
  }

  @Override
  public void setManualControl(boolean enabled) {
    manualControl = enabled;
  }

  @Override
  public boolean manualControlEnabled() {
    return manualControl;
  }

  @Override
  public void setManualVelocity(double normalizedVelocity) {
    if (manualControl) {
      leftMotor.set(normalizedVelocity);
    }
  }

  @Override
  public boolean atBottom() {
    // TODO: Make this better
    return getAngleDeg() <= bottomLimitDeg + 0.1;
  }

  @Override
  public boolean atTop() {
    // TODO: Make this better
    return getAngleDeg() >= topLimitDeg - 0.1;
  }

  @Override
  public void resetAngleToBottom() {

    leftMotor.getEncoder().setPosition(0.0);
    startingOffsetDeg = 0.0;
  }

  @Override
  public double getTopAngleDeg() {
    return topLimitDeg;
  }

  @Override
  public double getBottomAngleDeg() {
    return bottomLimitDeg;
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.positionRadsL = Units.rotationsToRadians(positionL) / reductionL;
    inputs.velocityRadsPerSecL =
        Units.rotationsToRadians(velocityL * (2 * Math.PI / 60)) / reductionL;
    inputs.appliedVoltageL = appliedVoltageL;
    inputs.outputCurrentAmpsL = outputCurrentL;
    inputs.tempCelsiusL = tempCelsiusL;

    inputs.positionRadsR = Units.rotationsToRadians(positionR) / reductionR;
    inputs.velocityRadsPerSecR =
        Units.rotationsToRadians(velocityR * (2 * Math.PI / 60)) / reductionR;
    inputs.appliedVoltageR = appliedVoltageR;
    inputs.outputCurrentAmpsR = outputCurrentR;
    inputs.tempCelsiusR = tempCelsiusR;
  }
}
