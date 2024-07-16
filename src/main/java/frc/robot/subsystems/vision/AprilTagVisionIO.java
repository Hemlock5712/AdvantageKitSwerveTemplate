// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import java.util.ArrayList;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface AprilTagVisionIO {
  class AprilTagVisionIOInputs implements LoggableInputs {

    ArrayList<PoseEstimate> poseEstimates = new ArrayList<>();

    @Override
    public void toLog(LogTable table) {
      table.put("poseEstimates", poseEstimates.size());
      for (PoseEstimate poseEstimate : poseEstimates) {
        int posePosition = poseEstimates.indexOf(poseEstimate);
        table.put(
            "pose/" + Integer.toString(posePosition),
            LimelightHelpers.pose2dToArray(poseEstimate.pose));
        table.put(
            "timestampSeconds/" + Integer.toString(posePosition), poseEstimate.timestampSeconds);
        table.put("tagCount/" + Integer.toString(posePosition), poseEstimate.tagCount);
        table.put("avgTagDist/" + Integer.toString(posePosition), poseEstimate.avgTagDist);
      }
      table.put("valid", !poseEstimates.isEmpty());
    }

    @Override
    public void fromLog(LogTable table) {
      int estimatedPoseCount = table.get("poseEstimates", 0);
      for (int i = 0; i < estimatedPoseCount; i++) {
        Pose2d pose =
            LimelightHelpers.toPose2D(table.get("pose/" + Integer.toString(i), new double[] {}));
        double timestampSeconds = table.get("timestampSeconds/" + Integer.toString(i), 0.0);
        int tagCount = table.get("tagCount/" + Integer.toString(i), 0);
        double avgTagDist = table.get("avgTagDist/" + Integer.toString(i), 0.0);
        PoseEstimate poseEstimate = new PoseEstimate();
        poseEstimate.pose = pose;
        poseEstimate.timestampSeconds = timestampSeconds;
        poseEstimate.tagCount = tagCount;
        poseEstimate.avgTagDist = avgTagDist;
        poseEstimates.add(poseEstimate);
      }
      table.get("valid", false);
    }
  }

  default void updateInputs(AprilTagVisionIOInputs inputs) {}
}
