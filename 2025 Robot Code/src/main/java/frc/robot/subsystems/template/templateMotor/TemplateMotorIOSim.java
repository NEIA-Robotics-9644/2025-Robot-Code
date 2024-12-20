package frc.robot.subsystems.template.templateMotor;

public class TemplateMotorIOSim implements TemplateMotorIO {

    private boolean newInput = false;

    private double normalizedVelocity = 0.0;

    private double positionRotations = 0.0;

    private double velocityRPM = 0.0;

    private double bottomLimitRotations = 0.0;

    private double topLimitRotations = 20.0;

    private double maxSpeedRPM = 10.0;

    private boolean inverted = false;

    private double measuredRPM = 0.0;

    private double lastPosition = 0.0;

    public TemplateMotorIOSim() {

    }

    @Override
    public void spinMotor(double normalizedVelocity) {
        this.normalizedVelocity = Math.max(-1.0, Math.min(1.0, normalizedVelocity));
        if (inverted) {
            this.normalizedVelocity *= -1;
        }
        newInput = true;
    }

    @Override
    public double getMotorVelocityRPM() {
        return measuredRPM;
    }

    @Override
    public void periodic() {
        lastPosition = positionRotations;

        if (newInput) {
            

            velocityRPM = normalizedVelocity * maxSpeedRPM;

            double delta = velocityRPM * 0.02; // 20ms

            if (positionRotations + delta < bottomLimitRotations) {
                positionRotations = bottomLimitRotations;
            } else if (positionRotations + delta > topLimitRotations) {
                positionRotations = topLimitRotations;
            } else {
                positionRotations += delta;
            }
            
            newInput = false;
        } else {
            velocityRPM = 0;
        }

        measuredRPM = (positionRotations - lastPosition) / 0.02;
    }

    @Override
    public double getMotorRotations() {
        return positionRotations;
    }
}