package frc.robot.subsystems.pivot;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class PivotIOSparkMax implements PivotIO {

  private final SparkMax motor;

  private final SparkMaxConfig config = new SparkMaxConfig();

  private double absoluteEncoderOffsetDegs = 0.0;

  public PivotIOSparkMax(int id) {
    this.motor = new SparkMax(id, SparkMax.MotorType.kBrushless);

    var config = new SparkMaxConfig();

    config.idleMode(SparkMaxConfig.IdleMode.kBrake);

    this.motor.configureAsync(
        config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  @Override
  public void setMaxAmps(int maxAmps) {
    config.smartCurrentLimit(maxAmps);
    this.motor.configureAsync(
        config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  @Override
  public void setInverted(boolean inverted) {
    config.inverted(inverted);
    this.motor.configureAsync(
        config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  @Override
  public void setVelocity(double normalizedVelocity) {
    motor.set(normalizedVelocity);
  }

  @Override
  public double getPositionRads() {
    var rawPosition = Rotation2d.fromRotations(motor.getAbsoluteEncoder().getPosition());
    var offsetPosition = rawPosition.plus(Rotation2d.fromDegrees(absoluteEncoderOffsetDegs));
    return offsetPosition.getRadians();
  }

  @Override
  public void setEncoderOffset(double encoderOffsetDegs) {
    this.absoluteEncoderOffsetDegs = encoderOffsetDegs;
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
