package frc.robot.subsystems.extender.pivot;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;

public class PivotIOSparkMax implements PivotIO {

  private final SparkMax motor;

  private SparkMaxConfig config = new SparkMaxConfig();

  private boolean manualControl = true;

  private PIDController pid = new PIDController(0, 0, 0);

  private double maxSpeed = 0.0;

  private double targetAngle = 0.0;

  public PivotIOSparkMax(
      int id, int maxCurrentA, double kP, double kD, double maxSpeed, boolean reversed) {
    this.motor = new SparkMax(id, SparkMax.MotorType.kBrushless);

    config.idleMode(SparkMaxConfig.IdleMode.kBrake);

    config.smartCurrentLimit(maxCurrentA);

    config.inverted(reversed);

    pid.setPID(kP, 0, kD);

    this.maxSpeed = maxSpeed;

    this.motor.configure(
        config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  @Override
  public void setAngleSetpoint(double angleDeg) {
    this.targetAngle = angleDeg / 360.0;
    this.manualControl = false;
  }

  @Override
  public void setManualVelocity(double normalizedVelocity) {
    motor.set(normalizedVelocity);
    this.manualControl = true;
  }

  @Override
  public void periodic() {

    if (!manualControl) {
      double error = motor.getEncoder().getPosition() - targetAngle;
      double output = pid.calculate(error);
      output = MathUtil.clamp(output, -maxSpeed, maxSpeed);
      motor.set(output);
    }
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.positionRads = Units.rotationsToRadians(motor.getEncoder().getPosition());
    inputs.velocityRadsPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(motor.getEncoder().getVelocity());
    inputs.appliedVoltage = motor.getAppliedOutput();
    inputs.outputCurrentAmps = motor.getOutputCurrent();
    inputs.tempCelsius = motor.getMotorTemperature();

    inputs.currentAngleDeg = motor.getEncoder().getPosition() * 360.0;
    inputs.targetAngleDeg = targetAngle * 360.0;
  }

  @Override
  public void zeroEncoder() {
    motor.getEncoder().setPosition(0.0);
  }
}
