package frc.robot.subsystems.extender.elevator;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;

public class ElevatorIOSparkMax implements ElevatorIO {

  private ControlMode mode = ControlMode.HOMING;

  private final SparkMax leftMotor;
  private final SparkMax rightMotor;

  private final boolean leftReversed = false;
  private final boolean rightReversed = true;

  private PIDController pid = new PIDController(2, 0.0, 0.0);
  private final double kG = 5;

  

  public ElevatorIOSparkMax(
      int leftMotorCanID,
      int rightMotorCanID,
      int currentLimit) {
    this.leftMotor = new SparkMax(leftMotorCanID, SparkMax.MotorType.kBrushless);
    this.rightMotor = new SparkMax(rightMotorCanID, SparkMax.MotorType.kBrushless);

    SparkMaxConfig lConfig = new SparkMaxConfig();
    SparkMaxConfig rConfig = new SparkMaxConfig();

    lConfig.inverted(leftReversed);

    rConfig.follow(leftMotorCanID, leftReversed != rightReversed);
    lConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
    rConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);

    lConfig.smartCurrentLimit(currentLimit);
    rConfig.smartCurrentLimit(currentLimit);

    this.leftMotor.configure(
        lConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    this.rightMotor.configure(
        rConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {

    // if (!manualControl) {

    //   // Calculate the pid
    //   double pidOutput = pid.calculate(getAngleDeg());

    //   double gravityComp = kG * Math.cos(Math.toRadians(getAngleDeg()));

    //   double output = pidOutput + gravityComp;

    //   // Clamp so it is under the max speed
    //   if (output > maxSpeedDegPerSec) {
    //     output = maxSpeedDegPerSec;
    //   } else if (output < -maxSpeedDegPerSec) {
    //     output = -maxSpeedDegPerSec;
    //   }

    //   this.leftMotor.set(output / physicalMaxSpeed);
    //   this.rightMotor.set(output / physicalMaxSpeed);
    // }
  }

  @Override
  public void setManualVelocity(double normalizedVelocity) {
    leftMotor.set(normalizedVelocity);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    var lEncoder = leftMotor.getEncoder();
    var rEncoder = rightMotor.getEncoder(); 

    inputs.positionRadsL = Units.rotationsToRadians(lEncoder.getPosition());
    inputs.velocityRadsPerSecL =
        Units.rotationsPerMinuteToRadiansPerSecond(lEncoder.getVelocity());
    inputs.appliedVoltageL = leftMotor.getAppliedOutput();
    inputs.outputCurrentAmpsL = leftMotor.getOutputCurrent();
    inputs.tempCelsiusL = leftMotor.getMotorTemperature();

    inputs.positionRadsR = Units.rotationsToRadians(rEncoder.getPosition());
    inputs.velocityRadsPerSecR =
        Units.rotationsPerMinuteToRadiansPerSecond(rEncoder.getVelocity());
    inputs.appliedVoltageR = rightMotor.getAppliedOutput();
    inputs.outputCurrentAmpsR = rightMotor.getOutputCurrent();
    inputs.tempCelsiusR = rightMotor.getMotorTemperature();

    inputs.switchTriggered = 
  }
}
