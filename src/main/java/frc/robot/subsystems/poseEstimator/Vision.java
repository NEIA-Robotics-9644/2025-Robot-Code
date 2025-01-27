package frc.robot.subsystems.poseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
  public final PhotonCamera camera;
  public final PhotonCamera camera2;
  private final PhotonPoseEstimator photonEstimator;
  private Matrix<N3, N1> curStdDevs;

  public static final AprilTagFieldLayout kTagLayout =
      AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();
  
  

  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

  public static final Transform3d kRobotToCam =
      new Transform3d(new Translation3d(Units.inchesToMeters(-5), Units.inchesToMeters(-4), Units.inchesToMeters(0)), new Rotation3d(0,Math.toRadians(30), 0));

  // Simulation
  private PhotonCameraSim cameraSim;
  private VisionSystemSim visionSim;

  public Vision() {
    camera = new PhotonCamera("Global_Shutter_Camera");
    camera2 = new PhotonCamera("Global_Shutter_Camera");

    photonEstimator =
        new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCam);
    photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    // ----- Simulation
    if (Robot.isSimulation()) {
      // Create the vision system simulation which handles cameras and targets on the field.
      visionSim = new VisionSystemSim("main");
      // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
      visionSim.addAprilTags(kTagLayout);
      // Create simulated camera properties. These can be set to mimic your actual camera.
      var cameraProp = new SimCameraProperties();
      cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
      cameraProp.setCalibError(0.35, 0.10);
      cameraProp.setFPS(30);
      cameraProp.setAvgLatencyMs(50);
      cameraProp.setLatencyStdDevMs(15);
      // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
      // targets.
      cameraSim = new PhotonCameraSim(camera, cameraProp);
      // Add the simulated camera to view the targets on this simulated field.
      visionSim.addCamera(cameraSim, kRobotToCam);

      cameraSim.enableDrawWireframe(true);
    }
  }

  /**
   * The latest estimated robot pose on the field from vision data. This may be empty. This should
   * only be called once per loop.
   *
   * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
   * {@link getEstimationStdDevs}
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
   *     used for estimation.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (var change : camera.getAllUnreadResults()) {
      visionEst = photonEstimator.update(change);
      updateEstimationStdDevs(visionEst, change.getTargets());

      if (Robot.isSimulation()) {
        visionEst.ifPresentOrElse(
            est ->
                getSimDebugField()
                    .getObject("VisionEstimation")
                    .setPose(est.estimatedPose.toPose2d()),
            () -> {
              getSimDebugField().getObject("VisionEstimation").setPoses();
            });
      }
    }
    return visionEst;
  }

  public Pose2d returnCameraToTarget(Vision vision) {

    PhotonPipelineResult result = camera.getLatestResult();

    if (result.hasTargets()) {
      // Get the best target
      var target = result.getBestTarget();

      // Get the camera-to-target Transform3d
      Transform3d cameraToTarget = target.getBestCameraToTarget();

      // Get the robot's current Pose3d (from your odometry or pose estimator)
      Pose3d robotPose = new Pose3d(/* Replace with your robot pose */ );

      // Combine the robot's Pose3d and camera-to-target Transform3d to get the field-relative pose
      Pose3d fieldPose = robotPose.transformBy(cameraToTarget);

      // Convert Pose3d to Pose2d (dropping z-axis information)
      Pose2d fieldPose2d =
          new Pose2d(fieldPose.getX(), fieldPose.getY(), fieldPose.getRotation().toRotation2d());
      return fieldPose2d;
    }
    else {
      return null;
    }
  }


  public Pose2d calculateRobotToTargetPose(Transform3d cameraToRobot) {
      // Get the latest result from the camera
      PhotonPipelineResult result = camera.getLatestResult();

      // Check if there are valid targets
      if (result.hasTargets()) {
          // Get the best target
          var target = result.getBestTarget();

          // Get the camera-to-target Transform3d
          Transform3d cameraToTarget = target.getBestCameraToTarget();

          // Calculate robot-to-target Transform3d
          Transform3d robotToTarget = cameraToRobot.plus(cameraToTarget);

          // Convert the robot-to-target Transform3d into a Pose3d
          Pose3d robotToTargetPose3d = new Pose3d(
              robotToTarget.getTranslation(),
              robotToTarget.getRotation()
          );

          // Convert Pose3d to Pose2d (dropping the z-axis and using only yaw for rotation)
          Pose2d robotToTargetPose2d = new Pose2d(
              robotToTargetPose3d.getX(),
              robotToTargetPose3d.getY(),
              new Rotation2d(robotToTargetPose3d.getRotation().getZ())
          );

          return robotToTargetPose2d;
      }

      // Return null if no targets are found
      else {return null;}
  }

  /**
   * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
   * deviations based on number of tags, estimation strategy, and distance from the tags.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets All targets in this camera frame
   */
  public PhotonPipelineResult returnTargets(PhotonCamera camera) {
    var result = camera.getLatestResult();
    return result;
  }

  private void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curStdDevs = kSingleTagStdDevs;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an average-distance metric
      for (var tgt : targets) {
        var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty()) continue;
        numTags++;
        avgDist +=
            tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        // No tags visible. Default to single-tag std devs
        curStdDevs = kSingleTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        curStdDevs = estStdDevs;
      }
    }
  }

  /**
   * Returns the latest standard deviations of the estimated pose from {@link
   * #getEstimatedGlobalPose()}, for use with {@link
   * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
   * only be used when there are targets visible.
   */
  public Matrix<N3, N1> getEstimationStdDevs() {
    return curStdDevs;
  }

  public void returnBestPose() {
    var result = camera.getLatestResult();
    boolean hasTargets = result.hasTargets();
    if (hasTargets) {
      PhotonTrackedTarget target = result.getBestTarget();
      double yaw = target.getYaw();
      double pitch = target.getPitch();
      double area = target.getArea();
      double skew = target.getSkew();
      int targetID = target.getFiducialId();
      double poseAmbiguity = target.getPoseAmbiguity();
      System.out.println(yaw);
      System.out.println(pitch);
      System.out.println(area);
      System.out.println(skew);
      System.out.println(targetID);
      System.out.println(poseAmbiguity);
    }
  }

  // ----- Simulation

  public void simulationPeriodic(Pose2d robotSimPose) {
    visionSim.update(robotSimPose);
  }

  /** Reset pose history of the robot in the vision system simulation. */
  public void resetSimPose(Pose2d pose) {
    if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
    if (!Robot.isSimulation()) return null;
    return visionSim.getDebugField();
  }

  public Quaternion getQuaternionFromAprilTags(List<PhotonTrackedTarget> targets) {
    // Ensure targets are not null or empty
    if (targets == null || targets.isEmpty()) {
      return null;
    }

    // Initialize cumulative rotation values
    double qx = 0;
    double qy = 0;
    double qz = 0;
    double qw = 0;
    int validTagCount = 0;

    // Iterate through targets to extract rotations
    for (PhotonTrackedTarget target : targets) {
      Optional<Pose3d> tagPose = photonEstimator.getFieldTags().getTagPose(target.getFiducialId());

      if (tagPose.isPresent()) {
        Rotation3d rotation = tagPose.get().getRotation();

        // Accumulate quaternion components
        qx += rotation.getQuaternion().getX();
        qy += rotation.getQuaternion().getY();
        qz += rotation.getQuaternion().getZ();
        qw += rotation.getQuaternion().getW();

        validTagCount++;
      }
    }

    // If no valid tags were found, return null
    if (validTagCount == 0) {
      return null;
    }

    // Normalize the accumulated quaternion
    double magnitude = Math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    qx /= magnitude;
    qy /= magnitude;
    qz /= magnitude;
    qw /= magnitude;

    // Return the quaternion object
    return new Quaternion(qx, qy, qz, qw);
  }

  @Override
  public void periodic() {
    if (camera.getLatestResult().hasTargets()) {
      System.out.println("target aquired");
      this.returnBestPose();
    }
  }
}
