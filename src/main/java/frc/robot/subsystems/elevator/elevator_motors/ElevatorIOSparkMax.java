package frc.robot.subsystems.elevator.elevator_motors;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;

public class ElevatorIOSparkMax implements ElevatorIO {

  private final SparkMax leftMotor;
  private final SparkMax rightMotor;

  private final SparkMaxConfig lConfig = new SparkMaxConfig();
  private final SparkMaxConfig rConfig = new SparkMaxConfig();

  public ElevatorIOSparkMax(
      int leftMotorCanID, int rightMotorCanID, boolean leftReversed, boolean rightReversed) {

    this.leftMotor = new SparkMax(leftMotorCanID, SparkMax.MotorType.kBrushless);
    this.rightMotor = new SparkMax(rightMotorCanID, SparkMax.MotorType.kBrushless);

    lConfig.inverted(leftReversed);

    rConfig.follow(leftMotor, leftReversed != rightReversed);
    lConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
    rConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);

    // This should be set by the subsystem, but if it isn't, this is a good default to be safe
    lConfig.smartCurrentLimit(10);
    rConfig.smartCurrentLimit(10);

    leftMotor.configure(
        lConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    rightMotor.configure(
        rConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
  }

  @Override
  public void setMaxAmps(int maxAmps) {

    lConfig.smartCurrentLimit(maxAmps);
    rConfig.smartCurrentLimit(maxAmps);

    leftMotor.configure(
        lConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    rightMotor.configure(
        rConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {}

  @Override
  public void setVelocity(double normalizedVelocity) {
    leftMotor.set(normalizedVelocity);
  }

  @Override
  public void zeroEncoders() {
    leftMotor.getEncoder().setPosition(0);
    rightMotor.getEncoder().setPosition(0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    var lEncoder = leftMotor.getEncoder();
    var rEncoder = rightMotor.getEncoder();

    inputs.positionRadsL = Units.rotationsToRadians(lEncoder.getPosition());
    inputs.velocityRadsPerSecL = Units.rotationsPerMinuteToRadiansPerSecond(lEncoder.getVelocity());
    inputs.appliedVoltageL = leftMotor.getAppliedOutput();
    inputs.outputCurrentAmpsL = leftMotor.getOutputCurrent();
    inputs.tempCelsiusL = leftMotor.getMotorTemperature();

    inputs.positionRadsR = Units.rotationsToRadians(rEncoder.getPosition());
    inputs.velocityRadsPerSecR = Units.rotationsPerMinuteToRadiansPerSecond(rEncoder.getVelocity());
    inputs.appliedVoltageR = rightMotor.getAppliedOutput();
    inputs.outputCurrentAmpsR = rightMotor.getOutputCurrent();
    inputs.tempCelsiusR = rightMotor.getMotorTemperature();
  }

  @Override
  public double getPositionRads() {
    return leftMotor.getEncoder().getPosition();
  }
}
