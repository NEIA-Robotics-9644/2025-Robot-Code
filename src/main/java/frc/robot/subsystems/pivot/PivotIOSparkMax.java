package frc.robot.subsystems.pivot;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;

public class PivotIOSparkMax implements PivotIO {

  private final SparkMax motor;

  private final SparkMaxConfig config = new SparkMaxConfig();

  public PivotIOSparkMax(int id) {
    this.motor = new SparkMax(id, SparkMax.MotorType.kBrushless);

    var config = new SparkMaxConfig();

    config.idleMode(SparkMaxConfig.IdleMode.kBrake);

    this.motor.configure(
        config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  @Override
  public void setMaxAmps(int maxAmps) {
    config.smartCurrentLimit(maxAmps);
    this.motor.configure(
        config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  @Override
  public void setInverted(boolean inverted) {
    config.inverted(inverted);
    this.motor.configure(
        config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  @Override
  public void setVelocity(double normalizedVelocity) {
    motor.set(normalizedVelocity);
  }

  @Override
  public double getPositionRads() {
    return Units.rotationsToRadians(motor.getEncoder().getPosition());
  }

  @Override
  public void setEncoderOffset(double encoderOffsetRads) {
    motor.getEncoder().setPosition(Units.radiansToRotations(encoderOffsetRads));
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.positionRads = Units.rotationsToRadians(motor.getEncoder().getPosition());
    inputs.velocityRadsPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(motor.getEncoder().getVelocity());
    inputs.appliedVoltage = motor.getAppliedOutput();
    inputs.outputCurrentAmps = motor.getOutputCurrent();
    inputs.tempCelsius = motor.getMotorTemperature();
  }

  @Override
  public void zeroEncoder() {
    motor.getEncoder().setPosition(0.0);
  }
}
