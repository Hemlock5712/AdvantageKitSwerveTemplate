// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface AprilTagVisionIO {
  public static class AprilTagVisionIOInputs implements LoggableInputs {

    public Pose3d estimatedPose = new Pose3d();
    public double captureTimestamp = 0.0;
    public boolean valid = false;
    public int[] currentTags = new int[] {};
    ;

    @Override
    public void toLog(LogTable table) {
      table.put("estimatedPose", estimatedPose);
      table.put("captureTimestamp", captureTimestamp);
      table.put("valid", valid);
      table.put("currentTags", currentTags);
    }

    @Override
    public void fromLog(LogTable table) {
      estimatedPose = table.get("estimatedPose", estimatedPose);
      captureTimestamp = table.get("latency", captureTimestamp);
      valid = table.get("valid", valid);
      currentTags = table.get("currentTags", currentTags);
    }
  }

  public default void updateInputs(AprilTagVisionIOInputs inputs) {}
}
