// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import lombok.Getter;
import lombok.Setter;

import java.util.ArrayList;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;

public interface AprilTagVisionIO {
  @Setter
  @Getter
  class AprilTagVisionIOInputs implements LoggableInputs {

    ArrayList<PoseEstimate> poseEstimates = new ArrayList<>();

    @Override
    public void toLog(LogTable table) {
      table.put("poseEstimates", poseEstimates.size());
      for (PoseEstimate poseEstimate : poseEstimates) {
        int posePosition = poseEstimates.indexOf(poseEstimate);
        table.put("estimatedPose/" + Integer.toString(posePosition),
            LimelightHelpers.getPose3dToArray(poseEstimate.getPose()));
        table.put("captureTimestamp/" + Integer.toString(posePosition), poseEstimate.getTimestampSeconds());
        table.put("tagIDs/" + Integer.toString(posePosition), poseEstimate.getTagIDs());
        table.put("averageTagDistance/" + Integer.toString(posePosition), poseEstimate.getAverageTagDistance());
      }
      table.put("valid", poseEstimates.isEmpty());
    }

    @Override
    public void fromLog(LogTable table) {
      int estimatedPoseCount = table.get("poseEstimates", 0);
      for (int i = 0; i < estimatedPoseCount; i++) {
        Pose3d poseEstimation = LimelightHelpers
            .toPose3D(table.get("estimatedPose/" + Integer.toString(i), new double[] {}));
        double timestamp = table.get("captureTimestamp/" + Integer.toString(i), 0.0);
        double averageTagDistance = table.get("averageTagDistance/" + Integer.toString(i), 0.0);
        int[] tagIDs = table.get("tagIDs/" + Integer.toString(i), new int[] {});
        poseEstimates.add(new PoseEstimate(poseEstimation, timestamp, averageTagDistance, tagIDs));
      }
      table.get("valid", false);
    }
  }

  default void updateInputs(AprilTagVisionIOInputs inputs) {
  }
}
