package frc.robot.utils;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.constants.VisionConstants;

public class PhotonBridge {
  private final AprilTagFieldLayout fieldLayout;
  private final Transform3d robotToCam = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
  private final PhotonCamera cam = new PhotonCamera(VisionConstants.PHOTON_CAMERA_NAME);
  private final PhotonPoseEstimator poseEstimator;

  // Simulation
  private VisionSystemSim visionSim;
  private SimCameraProperties camProps;
  private PhotonCameraSim camSim;

  public PhotonBridge() {
    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    if (RobotBase.isSimulation()) {
      visionSim = new VisionSystemSim(VisionConstants.PHOTON_CAMERA_NAME);
      visionSim.addAprilTags(fieldLayout);

      camProps = new SimCameraProperties();
      camProps.setCalibration(960, 720, Rotation2d.fromDegrees(90));
      camProps.setCalibError(0.25, 0.08);
      camProps.setFPS(45);
      camProps.setAvgLatencyMs(35);
      camProps.setLatencyStdDevMs(5);

      camSim = new PhotonCameraSim(cam, camProps);
      camSim.enableProcessedStream(true);

      // Our camera is mounted 0.1 meters forward and 0.5 meters up from the robot
      // pose,
      // (Robot pose is considered the center of rotation at the floor level, or Z =
      // 0)
      Translation3d robotToCameraTrl = new Translation3d(0.1, 0, 0.5);
      // and pitched 15 degrees up.
      Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-15), 0);
      Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

      // Add this camera to the vision system simulation with the given
      // robot-to-camera transform.
      visionSim.addCamera(camSim, robotToCamera);
    }
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    if (cam.isConnected()) {
      final var results = cam.getAllUnreadResults();
      if (results.isEmpty()) {
        return Optional.empty();
      }
      final var latestResult = results.get(results.size() - 1);
      return poseEstimator.update(latestResult);
    } else {
      // System.out.println("Photon Bridge Error: Camera Not Found");
      return Optional.empty();
    }
  }

  public PhotonPipelineResult getLatestPipelineResult() {
    return cam.getLatestResult();
  }

  public void reset() {
    reset(new Pose2d());
  }

  public void reset(Pose2d pose) {
    poseEstimator.setLastPose(pose);
    poseEstimator.setReferencePose(pose);
  }

  public void simulationPeriodic(Pose2d pose) {
    visionSim.update(pose);
  }
}
