package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.SwerveDriveSubsystem;

import java.util.function.Supplier;

/*
 * This is a command that will be used to drive the robot with a joystick
 * This command originated from the CTRE Phoenix Swerve Drive Generated Code
 */
public class JoystickDriveCmd extends Command {

    private final double MaxSpeed = DriveConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
    private final double MaxAngularRate = DriveConstants.kMaxAngularSpeedRadPerSec;

    private final SwerveDriveSubsystem driveSubsystem;


    private final Supplier<Double> forwardSupplier;
    private final Supplier<Double> sidwaysSupplier;
    private final Supplier<Double> rotationalSupplier;
    private final Supplier<Boolean> speedIncreaseSupplier;
    private final Supplier<Boolean> speedDecreaseSupplier;
    private final Supplier<Boolean> fieldOrientedSupplier;
    private final Supplier<Boolean> resetGyroSupplier;

    private boolean lastSpeedIncrease = false;
    private boolean lastSpeedDecrease = false;


    private final double[] speeds = {0.2, 0.6, 1.0};

    private int speedStepIndex = 0;

    private int speedsLength = speeds.length;


    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.005).withRotationalDeadband(MaxAngularRate * 0.005) // Add a 0.5% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop

    private final SwerveRequest.RobotCentric robotCentricDriveRequest = new SwerveRequest.RobotCentric()
        .withDeadband(MaxSpeed * 0.005).withRotationalDeadband(MaxAngularRate * 0.005)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);  
        
    
    
    
    public JoystickDriveCmd(SwerveDriveSubsystem SwerveDriveSubsystem, Supplier<Double> forward, Supplier<Double> sideways, Supplier<Double> rotation, Supplier<Boolean> speedIncrease, Supplier<Boolean> speedDecrease, Supplier<Boolean> fieldOriented, Supplier<Boolean> resetGyro) {
        
        driveSubsystem = SwerveDriveSubsystem;
        forwardSupplier = forward;
        sidwaysSupplier = sideways;
        rotationalSupplier = rotation;
        speedIncreaseSupplier = speedIncrease;
        speedDecreaseSupplier = speedDecrease;
        fieldOrientedSupplier = fieldOriented;
        resetGyroSupplier = resetGyro;

        

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(SwerveDriveSubsystem);


    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // Get the values
        double forward = forwardSupplier.get();
        double sideways = sidwaysSupplier.get();
        double rotation = rotationalSupplier.get();

        boolean speedIncrease = speedIncreaseSupplier.get() && !lastSpeedIncrease;
        boolean speedDecrease = speedDecreaseSupplier.get() && !lastSpeedDecrease;

        boolean fieldOriented = fieldOrientedSupplier.get();

        boolean resetGyro = resetGyroSupplier.get();

        lastSpeedIncrease = speedIncreaseSupplier.get();
        lastSpeedDecrease = speedDecreaseSupplier.get();

        if (speedIncrease && (speedStepIndex < speedsLength - 1)) {
            speedStepIndex++;            
        }

        if (speedDecrease && (speedStepIndex > 0)) {
            speedStepIndex--;
        }

        if (resetGyro) {
            driveSubsystem.tareEverything();
        }

        

        double speedMultiplier = speeds[speedStepIndex];

        

        // Adapt the values to the robot
        double forwardOutput = -Math.abs(forward) * forward * MaxSpeed * speedMultiplier;
        double sidewaysOutput = -Math.abs(sideways) * sideways * MaxSpeed * speedMultiplier;
        double rotationOutput = -Math.abs(rotation) * rotation * MaxAngularRate * speedMultiplier;
        

        
        if (fieldOriented) {
            driveSubsystem.setControl(
                    driveRequest.withVelocityX(forwardOutput)
                    .withVelocityY(sidewaysOutput)
                    .withRotationalRate(rotationOutput)
            );
        } else {
            driveSubsystem.setControl(
                    robotCentricDriveRequest.withVelocityX(forwardOutput)
                    .withVelocityY(sidewaysOutput)
                    .withRotationalRate(rotationOutput)
            );
        }
    }

    

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}