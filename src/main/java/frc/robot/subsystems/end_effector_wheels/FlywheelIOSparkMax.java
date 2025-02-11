package frc.robot.subsystems.end_effector_wheels;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class FlywheelIOSparkMax implements FlywheelIO {

  private final SparkMax sparkMax;


  private enum ControlMode {
    VOLTAGE,
    VELOCITY,
    POSITION
  }

  private ControlMode mode;

  public FlywheelIOSparkMax(int canId) {

    sparkMax = new SparkMax(canId, MotorType.kBrushless);

    var config = new SparkMaxConfig();

  }


  public void updateInputs(FlywheelIOInputs inputs) {

    inputs.appliedVolts = sparkMax.getAppliedOutput(); // I don't know if this is correct

    inputs.currentAmps = sparkMax.getOutputCurrent(); // I don't know if this is correct either

    inputs.relativeEncoderPosition = Rotation2d.fromRotations(sparkMax.getEncoder().getPosition());

    inputs.absoluteEncoderPosition = Rotation2d.fromRotations(sparkMax.getAbsoluteEncoder().getPosition());

    inputs.tempCelsius = sparkMax.getMotorTemperature();

    inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(sparkMax.getEncoder().getVelocity());
  }


  public void runVolts(double volts) {}

  public void runVelocity(double velocityRadPerSec) {}

  public void runPosition(Rotation2d position) {}

  public void setPID(double kP, double kI, double kD) {}

  public void setBrakeMode(boolean enabled) {}

  public void stop() {}

}
