package frc.robot;

import static frc.robot.Constants.CameraConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.CameraConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.Consumer;
import monologue.Logged;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class AprilTagVision implements Logged {
  private List<VisionCamera> cameras = new ArrayList<>();

  public AprilTagVision() {
    if (!Constants.isAlpha) {
      addCamera("ReefCam", CameraConstants.reefCamName, CameraConstants.reefCamTransform);
      // addCamera("CoralCam", CameraConstants.coralCamName, CameraConstants.coralCamTransform);
    }
  }

  public void addCamera(String name, String photonName, Transform3d robotToCamera) {
    var camera = new PhotonCamera(photonName);
    var estimator =
        new PhotonPoseEstimator(
            Constants.kOfficialField, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
    cameras.add(new VisionCamera(name, camera, estimator));
  }

  public void processVisionUpdates(Consumer<EstimatedRobotPose> poseConsumer, Pose2d curPose) {
    for (int i = 0; i < cameras.size(); i++) {
      var camera = cameras.get(i);
      var results = camera.camera().getAllUnreadResults();
      if (!results.isEmpty()) {
        var result = results.get(results.size() - 1);
        var estimatedPose = camera.estimator().update(result);
        if (estimatedPose.isPresent()) {
          log(camera.name() + "EstimatedPoseRaw", estimatedPose.get().estimatedPose);
          if (shouldAcceptUpdate(result, estimatedPose.get(), curPose)) {
            poseConsumer.accept(estimatedPose.get());
            log(camera.name() + "EstimatedPoseAccepted", estimatedPose.get().estimatedPose);
          }
        }
      }
      log(camera.name() + " Connected", camera.camera().isConnected());
    }
  }

  // Only reef tags
  private static final Set<Integer> allowedTags =
      Set.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);

  public boolean shouldAcceptUpdate(
      PhotonPipelineResult result, EstimatedRobotPose estimatedRobotPose, Pose2d curPose) {

    // Only accept reef tags
    if (!allowedTags.contains(estimatedRobotPose.targetsUsed.get(0).fiducialId)) {
      return false;
    }

    // Reject if off field
    if (!translationWithinField(estimatedRobotPose.estimatedPose.toPose2d().getTranslation())) {
      return false;
    }

    // Reject if > 6" above or below the ground
    if (Math.abs(estimatedRobotPose.estimatedPose.getZ()) > Units.inchesToMeters(6)) {
      return false;
    }

    // Reject if high ambiguity
    if (estimatedRobotPose.targetsUsed.get(0).poseAmbiguity > 0.5) {
      return false;
    }

    // Always accept if we have a multitag result and the size is large enough
    if (result.getMultiTagResult().isPresent()
        && estimatedRobotPose.targetsUsed.get(0).getArea() > 0.05) {
      return true;
    }

    var apriltagPose =
        Constants.kOfficialField.getTagPose(estimatedRobotPose.targetsUsed.get(0).getFiducialId());
    if (apriltagPose.isEmpty()) {
      return false;
    }

    // Reject if tag is too far away
    var distanceToTag =
        curPose.getTranslation().getDistance(apriltagPose.get().getTranslation().toTranslation2d());
    log("distance to tag", distanceToTag);

    if (distanceToTag > 7) {
      return false;
    }

    return true;
  }

  private boolean translationWithinBounds(
      Translation2d value, Translation2d min, Translation2d max) {
    return (value.getX() > min.getX() && value.getX() < max.getX())
        && (value.getY() > min.getY() && value.getY() < max.getY());
  }

  private boolean translationWithinField(Translation2d val) {
    return translationWithinBounds(
        val,
        new Translation2d(),
        new Translation2d(
            Constants.kOfficialField.getFieldLength(), Constants.kOfficialField.getFieldWidth()));
  }

  public static record VisionCamera(
      String name, PhotonCamera camera, PhotonPoseEstimator estimator) {}
}
