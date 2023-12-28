// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface AprilTagVisionIO {
  public static class AprilTagVisionIOInputs implements LoggableInputs {

    private double[][] estimatedPose = new double[][] {};
    private double[] captureTimestamp = new double[] {};
    private boolean valid = false;
    private int[] currentTags = new int[] {};

    public double[][] getEstimatedPose() {
      return estimatedPose;
    }

    public void setEstimatedPose(double[][] estimatedPose) {
      this.estimatedPose = estimatedPose;
    }

    public double[] getCaptureTimestamp() {
      return captureTimestamp;
    }

    public void setCaptureTimestamp(double[] captureTimestamp) {
      this.captureTimestamp = captureTimestamp;
    }

    public boolean isValid() {
      return valid;
    }

    public void setValid(boolean valid) {
      this.valid = valid;
    }

    public int[] getCurrentTags() {
      return currentTags;
    }

    public void setCurrentTags(int[] currentTags) {
      this.currentTags = currentTags;
    }

    @Override
    public void toLog(LogTable table) {
      table.put("estimatedPose", estimatedPose.length);
      for (int i = 0; i < estimatedPose.length; i++) {
        table.put("estimatedPose/" + Integer.toString(i), estimatedPose[i]);
      }
      table.put("captureTimestamp", captureTimestamp);
      table.put("valid", valid);
      table.put("currentTags", currentTags);
    }

    @Override
    public void fromLog(LogTable table) {
      int estimatedPoseCount = table.get("estimatedPose", 0);
      estimatedPose = new double[estimatedPoseCount][];
      for (int i = 0; i < estimatedPoseCount; i++) {
        estimatedPose[i] = table.get("estimatedPose/" + Integer.toString(i), new double[] {});
      }
      captureTimestamp = table.get("latency", captureTimestamp);
      valid = table.get("valid", valid);
      currentTags = table.get("currentTags", currentTags);
    }
  }

  public default void updateInputs(AprilTagVisionIOInputs inputs) {}
}
