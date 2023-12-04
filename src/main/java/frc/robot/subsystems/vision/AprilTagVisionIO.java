// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import java.util.ArrayList;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface AprilTagVisionIO {
  public static class AprilTagVisionIOInputs implements LoggableInputs {

    public Pose3d estimatedPose = new Pose3d();
    public double captureTimestamp = 0.0;
    public double numTags = 0;
    public boolean valid = false;
    private double[] blankPoseArray = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    public ArrayList<Double> currentTags = new ArrayList<Double>();

    private double[] toLogPose3(Pose3d p) {
      double[] poseArray = {
        p.getX(),
        p.getY(),
        p.getZ(),
        p.getRotation().getX(),
        p.getRotation().getY(),
        p.getRotation().getZ()
      };
      return poseArray;
    }

    private Pose3d fromLogPose3(double[] a) {
      Rotation3d r = new Rotation3d(a[3], a[4], a[5]);
      Pose3d pose = new Pose3d(a[0], a[1], a[2], r);
      return pose;
    }

    @Override
    public void toLog(LogTable table) {
      table.put("estimatedPose", toLogPose3(estimatedPose));
      table.put("captureTimestamp", captureTimestamp);
      table.put("numTags", numTags);
      table.put("valid", valid);
      table.put("currentTags", currentTags);
    }

    @Override
    public void fromLog(LogTable table) {
      estimatedPose = fromLogPose3(table.get("estimatedPose", blankPoseArray));
      captureTimestamp = table.get("latency", captureTimestamp);
      valid = table.get("valid", valid);
      numTags = table.get("numTags", numTags);
      currentTags = table.get("currentTags", currentTags);
    }
  }

  public default void updateInputs(AprilTagVisionIOInputs inputs) {}
}
