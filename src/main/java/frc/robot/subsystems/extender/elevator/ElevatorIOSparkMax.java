package frc.robot.subsystems.extender.elevator;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;

public class ElevatorIOSparkMax implements ElevatorIO {

  private ControlMode mode = ControlMode.MANUAL;

  private final SparkMax leftMotor;
  private final SparkMax rightMotor;

  private double kG = 0.0;

  private PIDController pid = new PIDController(0, 0, 0);

  private double targetHeight = 0.0;

  private double maxSpeedDownNormalized = 0.0;
  private double maxSpeedUpNormalized = 0.0;

  public ElevatorIOSparkMax(int leftMotorCanID, int rightMotorCanID, ElevatorIOConfig config) {

    this.leftMotor = new SparkMax(leftMotorCanID, SparkMax.MotorType.kBrushless);
    this.rightMotor = new SparkMax(rightMotorCanID, SparkMax.MotorType.kBrushless);

    this.configure(config);
  }

  @Override
  public void configure(ElevatorIOConfig config) {

    SparkMaxConfig lConfig = new SparkMaxConfig();
    SparkMaxConfig rConfig = new SparkMaxConfig();

    lConfig.inverted(config.leftReversed);

    rConfig.follow(leftMotor, config.leftReversed != config.rightReversed);
    lConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
    rConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);

    lConfig.smartCurrentLimit(config.maxCurrentA);

    pid.setPID(config.kP, 0, config.kD);

    this.kG = config.kG;

    this.maxSpeedDownNormalized = config.maxSpeedDownNormalized;
    this.maxSpeedUpNormalized = config.maxSpeedUpNormalized;

    this.leftMotor.configure(
        lConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    this.rightMotor.configure(
        rConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
  }

  public double getHeight() {
    return leftMotor.getEncoder().getPosition();
  }

  public double getTargetHeight() {
    return this.targetHeight;
  }

  @Override
  public void periodic() {

    switch (mode) {
      case MANUAL -> {}

      case SETPOINT -> {
        double output = pid.calculate(getHeight(), targetHeight) + kG;

        output = MathUtil.clamp(output, -maxSpeedDownNormalized, maxSpeedUpNormalized);

        leftMotor.set(output);
      }
    }
  }

  @Override
  public void setManualVelocity(double normalizedVelocity) {
    mode = ControlMode.MANUAL;
    leftMotor.set(normalizedVelocity);
  }

  @Override
  public void setSetpointHeight(double setpointHeight) {
    mode = ControlMode.SETPOINT;
    this.targetHeight = setpointHeight;
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

    inputs.mode = this.mode;
    inputs.currentHeight = lEncoder.getPosition();
    inputs.targetHeight = this.targetHeight;
  }
}
