// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.FieldConstants;
import frc.robot.util.VisionHelpers.PoseEstimate;
import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class AprilTagVisionIOPhotonVision implements AprilTagVisionIO {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonEstimator;

  private double lastEstTimestamp = 0;

  /**
   * Constructs a new AprilTagVisionIOPhotonVision instance.
   *
   * @param identifier The identifier of the PhotonCamera.
   * @param robotToCamera The transform from the robot's coordinate system to the camera's
   *     coordinate system.
   */
  public AprilTagVisionIOPhotonVision(
      String identifier, Transform3d robotToCamera) {
    camera = new PhotonCamera(identifier);
    photonEstimator =
        new PhotonPoseEstimator(
            FieldConstants.aprilTags,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera,
            robotToCamera);
    photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  /**
   * Updates the inputs for AprilTag vision.
   *
   * @param inputs The AprilTagVisionIOInputs object containing the inputs.
   */
  @Override
  public void updateInputs(AprilTagVisionIOInputs inputs) {
    PhotonPipelineResult results = camera.getLatestResult();
    ArrayList<PoseEstimate> poseEstimates = new ArrayList<>();
    double timestamp = results.getTimestampSeconds();
    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
    if (!results.targets.isEmpty() && allianceOptional.isPresent()) {
      double latencyMS = results.getLatencyMillis();
      Pose3d poseEstimation;
      Optional<EstimatedRobotPose> estimatedPose = getEstimatedGlobalPose();
      if (estimatedPose.isEmpty()) {
        return;
      }
      poseEstimation = estimatedPose.get().estimatedPose;
      double averageTagDistance = 0.0;
      timestamp -= (latencyMS / 1e3);
      int[] tagIDs = new int[results.targets.size()];
      for (int i = 0; i < results.targets.size(); i++) {
        tagIDs[i] = results.targets.get(i).getFiducialId();
        var tagPose = photonEstimator.getFieldTags().getTagPose(tagIDs[i]);
        if (tagPose.isEmpty()) {
          continue;
        }
        averageTagDistance +=
            tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(poseEstimation.getTranslation().toTranslation2d());
      }
      averageTagDistance /= tagIDs.length;
      poseEstimates.add(new PoseEstimate(poseEstimation, timestamp, averageTagDistance, tagIDs));
      inputs.poseEstimates = poseEstimates;
    }
  }

  /** Updates the PhotonPoseEstimator and returns the estimated global pose. */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    var visionEst = photonEstimator.update();
    double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
    boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
    if (newResult) lastEstTimestamp = latestTimestamp;
    return visionEst;
  }
}
