package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.poseEstimator.Vision;

/** Tracks the robot's position, orientation, and field-relative motion. */
public class RobotState {
  private static RobotState instance = null;

  public Vision vision = new Vision();

  // Current pose of the robot on the field
  private Pose2d currentPose = new Pose2d();

  // Field-relative velocity
  private ChassisSpeeds fieldVelocity = new ChassisSpeeds();

  // Twist representation of movement
  private Twist2d fieldTwist = new Twist2d();

  // Swerve module positions
  private SwerveModulePosition[] modulePositions;

  // Drive kinematics
  private final SwerveDriveKinematics kinematics;

  /**
   * Singleton instance retrieval.
   *
   * @return The singleton instance of RobotState.
   */
  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState(DriveConstants.kinematics);
    }
    return instance;
  }

  /**
   * Constructs a RobotState with the specified kinematics.
   *
   * @param kinematics SwerveDriveKinematics for the robot's drivetrain.
   */
  private RobotState(SwerveDriveKinematics kinematics) {
    this.kinematics = kinematics;
    this.modulePositions = new SwerveModulePosition[4];
    for (int i = 0; i < modulePositions.length; i++) {
      modulePositions[i] = new SwerveModulePosition(0.0, new Rotation2d());
    }
  }

  /**
   * Updates the robot's pose, field-relative velocity, and field-relative twist.
   *
   * @param pose The new pose of the robot.
   * @param chassisSpeeds The field-relative chassis speeds of the robot.
   * @param modulePositions The current swerve module positions.
   * @param dt The time step in seconds since the last update.
   */
  public void update(
      Pose2d pose, ChassisSpeeds chassisSpeeds, SwerveModulePosition[] modulePositions, double dt) {
    this.currentPose = pose;
    this.fieldVelocity = chassisSpeeds;

    // Compute field-relative twist
    double dx = chassisSpeeds.vxMetersPerSecond * dt;
    double dy = chassisSpeeds.vyMetersPerSecond * dt;
    double dtheta = chassisSpeeds.omegaRadiansPerSecond * dt;
    this.fieldTwist = new Twist2d(dx, dy, dtheta);

    this.modulePositions = modulePositions.clone();
  }

  /**
   * Gets the robot's estimated pose on the field.
   *
   * @return The current pose of the robot.
   */
  public Pose2d getEstimatedPose(Drive drive) {
    this.currentPose = drive.getPose();
    return currentPose;
  }

  /**
   * Gets the field-relative velocity of the robot.
   *
   * @return The field-relative chassis speeds.
   */
  public ChassisSpeeds getFieldVelocity() {
    return fieldVelocity;
  }

  /**
   * Gets the field-relative twist of the robot.
   *
   * @return The field-relative twist (dx, dy, dtheta).
   */
  public Twist2d getFieldTwist() {
    return fieldTwist;
  }

  /**
   * Gets the current swerve module positions.
   *
   * @return An array of swerve module positions.
   */
  public SwerveModulePosition[] getModulePositions() {
    return modulePositions.clone();
  }

  /**
   * Resets the robot's pose on the field.
   *
   * @param newPose The new pose to set for the robot.
   */
  public void resetPose(Pose2d newPose) {
    this.currentPose = newPose;
  }

  /**
   * Gets the swerve drive kinematics for the robot.
   *
   * @return The swerve drive kinematics object.
   */
  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  /**
   * Checks if the robot is on the field and enabled.
   *
   * @return True if the robot is on the field and enabled.
   */
  public boolean isRobotEnabled() {
    return DriverStation.isEnabled();
  }
}
