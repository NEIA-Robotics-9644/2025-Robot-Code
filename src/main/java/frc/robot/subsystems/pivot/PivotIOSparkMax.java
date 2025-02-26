package frc.robot.subsystems.pivot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.SparkMaxUtil;

public class PivotIOSparkMax implements PivotIO {

  private final SparkMax motor;

  private final SparkMaxConfig config = new SparkMaxConfig();

  private double absoluteEncoderOffsetRads = 0.0;

  public PivotIOSparkMax(int id) {
    this.motor = new SparkMax(id, SparkMax.MotorType.kBrushless);

    var config = new SparkMaxConfig();

    config.idleMode(SparkMaxConfig.IdleMode.kBrake);

    SparkMaxUtil.configureMotor(motor, config);
  }

  @Override
  public void setMaxAmps(int maxAmps) {
    config.smartCurrentLimit(maxAmps);
    SparkMaxUtil.configureMotor(motor, config);
  }

  @Override
  public void setInverted(boolean inverted) {
    config.inverted(inverted);
    SparkMaxUtil.configureMotor(motor, config);
  }

  @Override
  public void setVelocity(double normalizedVelocity) {
    motor.set(normalizedVelocity);
  }

  @Override
  public double getPositionRads() {
    var rawPosition = Rotation2d.fromRotations(motor.getAbsoluteEncoder().getPosition());
    var offsetPosition = rawPosition.plus(Rotation2d.fromRadians(absoluteEncoderOffsetRads));
    return offsetPosition.getRadians();
  }

  @Override
  public void setEncoderOffset(double encoderOffsetRads) {
    this.absoluteEncoderOffsetRads = encoderOffsetRads;
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.positionRads = getPositionRads();
    inputs.velocityRadsPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(motor.getAbsoluteEncoder().getVelocity());
    inputs.appliedVoltage = motor.getAppliedOutput();
    inputs.outputCurrentAmps = motor.getOutputCurrent();
    inputs.tempCelsius = motor.getMotorTemperature();
  }

  @Override
  public void zeroEncoder() {
    // No-op because the encoder is absolute
  }
}
