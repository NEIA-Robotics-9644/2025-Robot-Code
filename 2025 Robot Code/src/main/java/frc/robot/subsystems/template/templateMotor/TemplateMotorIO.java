package frc.robot.subsystems.template.templateMotor;

public interface TemplateMotorIO {
    public void spinMotor(double normalizedVelocity);

    public double getMotorRotations();

    public double getMotorVelocityRPM();

    public void periodic();
}
