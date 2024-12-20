package frc.robot.subsystems.template.templateMotor;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;


public class TemplateMotorIOSparkMax implements TemplateMotorIO {
    
    private final CANSparkMax motor;

    private boolean newInput = false;

    private double velocity = 0.0;



    public TemplateMotorIOSparkMax(int canID) {
        this.motor = new CANSparkMax(canID, MotorType.kBrushless);
        motor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void spinMotor(double normalizedVelocity) {
        this.velocity = Math.max(-1.0, Math.min(1.0, normalizedVelocity * 2));
        newInput = true;
    }

    @Override
    public double getMotorVelocityRPM() {
        return motor.getEncoder().getVelocity() / 2;
    }

    @Override
    public void periodic() {
        if (newInput) {
            
            motor.set(velocity);
            newInput = false;
        } else {
            motor.set(0.0);
        }
    }

    @Override
    public double getMotorRotations() {
        return motor.getEncoder().getPosition() / 2;
    }
}