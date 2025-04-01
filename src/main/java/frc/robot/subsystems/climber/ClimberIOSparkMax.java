package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.SparkMaxUtil;

public class ClimberIOSparkMax implements ClimberIO {
  private final SparkMax motor;

  private final SparkMaxConfig config = new SparkMaxConfig();

  public ClimberIOSparkMax(int id) {
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
  public void setVelocity(double normalizedVelocity) {
    motor.set(normalizedVelocity);
  }

  @Override
  public void setMotorInverted(boolean inverted) {
    config.inverted(inverted);
    SparkMaxUtil.configureMotor(motor, config);
  }

  @Override
  public double getPositionRads() {
    var rawPosition = Rotation2d.fromRotations(motor.getEncoder().getPosition());
    var offsetPosition = rawPosition;
    return offsetPosition.getRadians();
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.positionRads = getPositionRads();
    inputs.velocityRadsPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(motor.getEncoder().getVelocity());
    inputs.appliedVoltage = motor.getAppliedOutput();
    inputs.outputCurrentAmps = motor.getOutputCurrent();
    inputs.tempCelsius = motor.getMotorTemperature();
  }

  @Override
  public void zeroEncoder() {
    motor.getEncoder().setPosition(0);
  }
}
