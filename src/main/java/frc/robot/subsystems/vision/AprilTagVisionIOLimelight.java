// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.TimestampedString;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.VisionHelpers.PoseEstimate;
import java.util.ArrayList;
import java.util.Optional;

public class AprilTagVisionIOLimelight implements AprilTagVisionIO {

  String limelightName;
  private final StringSubscriber observationSubscriber;

  public AprilTagVisionIOLimelight(String identifier) {
    NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable(identifier);
    LimelightHelpers.setPipelineIndex(limelightName, 0);

    observationSubscriber =
        limelightTable
            .getStringTopic("json")
            .subscribe("", PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
  }

  @Override
  public void updateInputs(AprilTagVisionIOInputs inputs) {
    TimestampedString[] queue = observationSubscriber.readQueue();
    ArrayList<PoseEstimate> poseEstimates = new ArrayList<>();
    for (TimestampedString timestampedString : queue) {
      double timestamp = timestampedString.timestamp / 1e6;
      LimelightHelpers.Results results =
          LimelightHelpers.parseJsonDump(timestampedString.value).targetingResults;
      Optional<Alliance> allianceOptional = DriverStation.getAlliance();
      if (results.targets_Fiducials.length == 0 || !allianceOptional.isPresent()) {
        continue;
      }
      double latencyMS = results.latency_capture + results.latency_pipeline;
      Pose3d poseEstimation = results.getBotPose3d_wpiBlue();
      double averageTagDistance = 0.0;
      timestamp -= (latencyMS / 1e3);
      int[] tagIDs = new int[results.targets_Fiducials.length];
      for (int i = 0; i < results.targets_Fiducials.length; i++) {
        tagIDs[i] = (int) results.targets_Fiducials[i].fiducialID;
        averageTagDistance +=
            results.targets_Fiducials[i].getTargetPose_CameraSpace().getTranslation().getNorm();
      }
      averageTagDistance /= tagIDs.length;
      poseEstimates.add(new PoseEstimate(poseEstimation, timestamp, averageTagDistance, tagIDs));
    }
    inputs.poseEstimates = poseEstimates;
  }
}
