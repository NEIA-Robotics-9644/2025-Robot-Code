package frc.robot.subsystems.drive;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class SwerveDriveSubsystem extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private BooleanSupplier isRed = () -> false;

    private final VisionIO visionIO;

    public SwerveDriveSubsystem(SwerveDrivetrainConstants driveTrainConstants, VisionIO visionIO, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        if (visionIO == null) {
            throw new IllegalArgumentException("VisionIO cannot be null");
        }

        this.visionIO = visionIO;

        //initializePathPlanner();

        
        setCurrentLimit(Constants.DriveConstants.kSupplyCurrentA);

        resetPose(new Pose2d(0, 0, new Rotation2d(0.0)));
    }
    public SwerveDriveSubsystem(SwerveDrivetrainConstants driveTrainConstants, VisionIO visionIO, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        if (visionIO == null) {
            throw new IllegalArgumentException("VisionIO cannot be null");
        }

        this.visionIO = visionIO;

        //initializePathPlanner();

        setCurrentLimit(Constants.DriveConstants.kSupplyCurrentA);

        
        resetPose(new Pose2d(0, 0, new Rotation2d(0.0)));
    }

    private void setCurrentLimit(double supplyCurrentLimit) {

        for (int i = 0; i < Modules.length; i++) {
            final int index = i;
            Shuffleboard.getTab("Current").addDouble("Motor " + i + " Output Current", () -> Modules[index].getDriveMotor().getSupplyCurrent().getValueAsDouble());
        }
        
        for (SwerveModule module : Modules) {
            module.getDriveMotor().getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(supplyCurrentLimit).withStatorCurrentLimitEnable(true).withStatorCurrentLimit(supplyCurrentLimit));
        }
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    // public void initializePathPlanner() {
    //     AutoBuilder.configureHolonomic(
    //         this::getPose, // Robot pose supplier
    //         this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
    //         this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //         this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
    //         new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
    //                 new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
    //                 new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
    //                 2, // Max module speed, in m/s
    //                 Constants.PhysicalRobotCharacteristics.kDriveBaseRadiusMeters, // Drive base radius in meters. Distance from robot center to furthest module.
    //                 new ReplanningConfig() // Default path replanning config. See the API for the options here
    //         ),
    //         isRed,
    //         this // Reference to this subsystem to set requirements
    //     );
    // }

    public Pose2d getPose() {
        return this.m_odometry.getEstimatedPosition();
    }

    public Consumer<Pose2d> resetPose(Pose2d pose2d) {
        this.getPigeon2().reset();
        this.m_odometry.resetPosition(new Rotation2d(0.0), new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        },
        pose2d);
        return pose -> this.m_odometry.resetPosition(new Rotation2d(0.0), new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        },
        pose2d);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
    
        return this.m_kinematics.toChassisSpeeds(this.getState().ModuleStates);
    
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        this.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(speeds));
    }



    @SuppressWarnings("resource")
    @Override
    public void periodic() {
        var visionResult = visionIO.getEstimatedGlobalPose();


        if (visionResult.isPresent()) {

            // Make a pose2d from the pose3d
            var translation = visionResult.get().estimatedPose.getTranslation();
            var rotation = visionResult.get().estimatedPose.getRotation();
            var pose = new Pose2d(translation.getX(), translation.getY(), new Rotation2d(rotation.getZ()));

            addVisionMeasurement(pose, visionResult.get().timestampSeconds, visionIO.getEstimationStdDevs(pose));
        }


        SmartDashboard.putString("Pose", this.m_odometry.getEstimatedPosition().toString());

        SmartDashboard.putNumber("Gyro Angle (Degrees)", this.getPigeon2().getAngle());

        

    }

    public void setFieldSide(boolean isRed) {
        this.isRed = () -> isRed;
    }
    
}
