package frc.robot.subsystems.intake.wheel;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;

public class IntakeWheelIOSparkMax implements IntakeWheelIO {

  private final SparkMax motor;

  private SparkMaxConfig config = new SparkMaxConfig();

  private boolean newInput = false;

  private final double reduction;

  private double normalizedVelocity = 0.0;

  final int maxCurrentA = 40;

  private final Double position;
  private final Double velocity;
  private final Double appliedVoltage;
  private final Double outputCurrent;
  private final Double tempCelsius;

  public IntakeWheelIOSparkMax(int id, double reduction) {
    this.motor = new SparkMax(id, SparkMax.MotorType.kBrushless);

    config.idleMode(SparkMaxConfig.IdleMode.kBrake);

    config.smartCurrentLimit(maxCurrentA);
    config.openLoopRampRate(0.5);

    this.motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

    this.reduction = reduction;

    position = motor.getAbsoluteEncoder().getPosition();
    velocity = motor.getAbsoluteEncoder().getVelocity() / 2;
    appliedVoltage = motor.getBusVoltage();
    outputCurrent = motor.getOutputCurrent();
    tempCelsius = motor.getMotorTemperature();
  }

  @Override
  public void setVelocity(double normalizedVelocity) {
    this.normalizedVelocity = Math.max(-1.0, Math.min(1.0, normalizedVelocity));
    newInput = true;
  }

  private double maxSpeedRPM = 5000.0;

  @Override
  public double getVelocityPercent() {
      return motor.getEncoder().getVelocity() / maxSpeedRPM;
  }

  @Override
  public void periodic() {
    if (newInput) {
      motor.set(normalizedVelocity);
      newInput = false;
    } else {
        motor.set(0.0);
    }
  }

  @Override
  public void updateInputs(IntakeWheelIOInputs inputs) {
    inputs.positionRads = Units.rotationsToRadians(position) / reduction;
    inputs.velocityRadsPerSec = Units.rotationsToRadians(velocity * (2*Math.PI/60)) / reduction;
    inputs.appliedVoltage = appliedVoltage;
    inputs.outputCurrentAmps = outputCurrent;
    inputs.tempCelsius = tempCelsius;
  }

  @Override
  public void setBrakeMode(boolean brake) {
      config.idleMode(brake ? SparkMaxConfig.IdleMode.kBrake : SparkMaxConfig.IdleMode.kCoast);
      this.motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
  }
}