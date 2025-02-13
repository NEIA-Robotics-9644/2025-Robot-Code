/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot.subsystems.poseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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
  public final PhotonCamera camera_front;
  public final PhotonCamera camera_back;
  private final PhotonPoseEstimator backPhotonEstimator;
  private final PhotonPoseEstimator frontPhotonEstimator;

  public static final AprilTagFieldLayout kTagLayout =
      AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();

  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

  private Matrix<N3, N1> curBackStdDevs = kSingleTagStdDevs;
  private Matrix<N3, N1> curFrontStdDevs = kSingleTagStdDevs;

  public static final Transform3d kBackRobotToCam =
      new Transform3d(
          new Translation3d(-0.301, 0.184, -0.249), new Rotation3d(0, 0, Math.toRadians(0)));
  public static final Transform3d kFrontRobotToCam =
      new Transform3d(
          new Translation3d(0.321, 0.184, -0.249), new Rotation3d(0, 0, Math.toRadians(0)));

  // Simulation
  private PhotonCameraSim frontCameraSim;
  private PhotonCameraSim backCameraSim;
  public VisionSystemSim visionSim;

  public Vision() {
    camera_front = new PhotonCamera("camera_front");
    camera_back = new PhotonCamera("camera_back");

    backPhotonEstimator =
        new PhotonPoseEstimator(
            kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kBackRobotToCam);
    backPhotonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    frontPhotonEstimator =
        new PhotonPoseEstimator(
            kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kFrontRobotToCam);
    frontPhotonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

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
      frontCameraSim = new PhotonCameraSim(camera_front, cameraProp);
      backCameraSim = new PhotonCameraSim(camera_back, cameraProp);
      // Add the simulated camera to view the targets on this simulated field.
      visionSim.addCamera(frontCameraSim, kFrontRobotToCam);
      visionSim.addCamera(backCameraSim, kBackRobotToCam);

      frontCameraSim.enableDrawWireframe(true);
      backCameraSim.enableDrawWireframe(true);
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
  public Optional<EstimatedRobotPose> getBackEstimatedGlobalPose() {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (var change : camera_back.getAllUnreadResults()) {
      visionEst = backPhotonEstimator.update(change);
      updateBackEstimationStdDevs(visionEst, change.getTargets());

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

  public Optional<EstimatedRobotPose> getFrontEstimatedGlobalPose() {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (var change : camera_front.getAllUnreadResults()) {
      visionEst = frontPhotonEstimator.update(change);
      updateFrontEstimationStdDevs(visionEst, change.getTargets());

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

  private void updateBackEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curBackStdDevs = kSingleTagStdDevs;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an average-distance metric
      for (var tgt : targets) {
        var tagPose = backPhotonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
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
        curBackStdDevs = kSingleTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        curBackStdDevs = estStdDevs;
      }
    }
  }

  private void updateFrontEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curFrontStdDevs = kSingleTagStdDevs;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an average-distance metric
      for (var tgt : targets) {
        var tagPose = frontPhotonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
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
        curFrontStdDevs = kSingleTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        curFrontStdDevs = estStdDevs;
      }
    }
  }

  /**
   * Returns the latest standard deviations of the estimated pose from {@link
   * #getEstimatedGlobalPose()}, for use with {@link
   * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
   * only be used when there are targets visible.
   */
  public Matrix<N3, N1> getFrontEstimationStdDevs() {
    return curFrontStdDevs;
  }

  public Matrix<N3, N1> getBackEstimationStdDevs() {
    return curBackStdDevs;
  }

  public void returnBestPose() {
    var result = camera_front.getLatestResult();
    boolean hasTargets = result.hasTargets();
    if (hasTargets) {
      PhotonTrackedTarget target = result.getBestTarget();
      double yaw = target.getYaw();
      double pitch = target.getPitch();
      double area = target.getArea();
      double skew = target.getSkew();
      int targetID = target.getFiducialId();
      double poseAmbiguity = target.getPoseAmbiguity();
      // System.out.println(yaw);
      // System.out.println(pitch);
      // System.out.println(area);
      // System.out.println(skew);
      // System.out.println(targetID);
      // System.out.println(poseAmbiguity);
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

  // public Quaternion getQuaternionFromAprilTags(List<PhotonTrackedTarget> targets) {
  //   // Ensure targets are not null or empty
  //   if (targets == null || targets.isEmpty()) {
  //     return null;
  //   }

  //   // Initialize cumulative rotation values
  //   double qx = 0;
  //   double qy = 0;
  //   double qz = 0;
  //   double qw = 0;
  //   int validTagCount = 0;

  //   // Iterate through targets to extract rotations
  //   for (PhotonTrackedTarget target : targets) {
  //     Optional<Pose3d> tagPose =
  // photonEstimator.getFieldTags().getTagPose(target.getFiducialId());

  //     if (tagPose.isPresent()) {
  //       Rotation3d rotation = tagPose.get().getRotation();

  //       // Accumulate quaternion components
  //       qx += rotation.getQuaternion().getX();
  //       qy += rotation.getQuaternion().getY();
  //       qz += rotation.getQuaternion().getZ();
  //       qw += rotation.getQuaternion().getW();

  //       validTagCount++;
  //     }
  //   }

  // // If no valid tags were found, return null
  // if (validTagCount == 0) {
  //   return null;
  // }

  //   // Normalize the accumulated quaternion
  //   double magnitude = Math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  //   qx /= magnitude;
  //   qy /= magnitude;
  //   qz /= magnitude;
  //   qw /= magnitude;

  //   // Return the quaternion object
  //   return new Quaternion(qx, qy, qz, qw);
  // }

  @Override
  public void periodic() {
    this.returnBestPose();
  }
}
