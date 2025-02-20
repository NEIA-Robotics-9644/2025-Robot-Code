package frc.robot.subsystems.end_effector; // volatge, velo, pos

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class FlywheelIOSparkMax implements FlywheelIO {

  private final SparkMax motor;

  private final SparkMaxConfig config = new SparkMaxConfig();

  public FlywheelIOSparkMax(int id) {
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
  public void updateInputs(FlywheelIOInputs inputs) {

    inputs.appliedVolts = motor.getAppliedOutput(); // I don't know if this is correct

    inputs.currentAmps = motor.getOutputCurrent(); // I don't know if this is correct either

    inputs.relativeEncoderPosition = Rotation2d.fromRotations(motor.getEncoder().getPosition());

    inputs.absoluteEncoderPosition =
        Rotation2d.fromRotations(motor.getAbsoluteEncoder().getPosition());

    inputs.tempCelsius = motor.getMotorTemperature();

    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(motor.getEncoder().getVelocity());
  }

  @Override
  public void zeroEncoder() {
    motor.getEncoder().setPosition(0.0);
  }
}
