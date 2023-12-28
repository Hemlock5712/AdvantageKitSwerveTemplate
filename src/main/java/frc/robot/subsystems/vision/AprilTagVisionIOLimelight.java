// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import java.util.concurrent.atomic.AtomicReference;

public class AprilTagVisionIOLimelight implements AprilTagVisionIO {

  String limelightName;
  AtomicReference<Alliance> alliance = new AtomicReference<>();
  private final DoubleArraySubscriber observationSubscriber;

  public AprilTagVisionIOLimelight(String identifier) {
    NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable(identifier);
    LimelightHelpers.setPipelineIndex(limelightName, 0);
    alliance.set(DriverStation.getAlliance().orElse(Alliance.Blue));
    if (alliance.get() == Alliance.Blue) {
      observationSubscriber =
          limelightTable
              .getDoubleArrayTopic("botpose_wpiblue")
              .subscribe(
                  new double[] {}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
    } else {
      observationSubscriber =
          limelightTable
              .getDoubleArrayTopic("botpose_wpired")
              .subscribe(
                  new double[] {}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
    }
  }

  @Override
  public void updateInputs(AprilTagVisionIOInputs inputs) {
    TimestampedDoubleArray[] queue = observationSubscriber.readQueue();
    double[] captureTimestamp = new double[queue.length];
    double[][] estimatedPose = new double[queue.length][];
    for (int i = 0; i < queue.length; i++) {
      PoseEstimate poseEstimate = LimelightHelpers.queueToPoseEstimate(queue, i);
      captureTimestamp[i] = poseEstimate.timestampSeconds;
      estimatedPose[i] = LimelightHelpers.poseToArray(poseEstimate.pose);
    }
    LimelightTarget_Fiducial[] tagID =
        LimelightHelpers.getLatestResults(limelightName).targetingResults.targets_Fiducials;
    int[] temp = new int[tagID.length];
    for (int i = 0; i < tagID.length; i++) {
      temp[i] = (int) tagID[i].fiducialID;
    }
    inputs.setCurrentTags(temp);
    inputs.setValid(LimelightHelpers.getTV(limelightName));
    inputs.setCaptureTimestamp(captureTimestamp);
    inputs.setEstimatedPose(estimatedPose);
  }
}
