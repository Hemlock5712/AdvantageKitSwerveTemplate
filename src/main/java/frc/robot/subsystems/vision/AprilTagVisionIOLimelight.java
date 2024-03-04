// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.VisionHelpers.PoseEstimate;
import java.util.ArrayList;
import java.util.Optional;

/** This class represents the implementation of AprilTagVisionIO using Limelight camera. */
public class AprilTagVisionIOLimelight implements AprilTagVisionIO {

  String limelightName;
  DoubleArraySubscriber observationSubscriber;

  /**
   * Constructs a new AprilTagVisionIOLimelight instance.
   *
   * @param identifier The identifier of the Limelight camera.
   */
  public AprilTagVisionIOLimelight(String identifier) {
    NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable(identifier);
    LimelightHelpers.setPipelineIndex(limelightName, 0);

    observationSubscriber =
        limelightTable
            .getDoubleArrayTopic("botpose_wpiblue")
            .subscribe(
                new double[] {}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
  }

  /**
   * Updates the inputs for AprilTag vision.
   *
   * @param inputs The AprilTagVisionIOInputs object containing the inputs.
   */
  @Override
  public void updateInputs(AprilTagVisionIOInputs inputs) {
    ArrayList<PoseEstimate> poseEstimates =
        new ArrayList<>(); // Creates an empty ArrayList to store pose estimates

    TimestampedDoubleArray[] queue =
        new TimestampedDoubleArray[] {observationSubscriber.getAtomic()};

    for (int i = 0; i < queue.length; i++) {

      Optional<Alliance> allianceOptional =
          DriverStation.getAlliance(); // Retrieves the alliance information from the DriverStation
      TimestampedDoubleArray timestampedDouble = queue[i];
      double timestamp = timestampedDouble.timestamp / 1e6; // Converts the timestamp to seconds
      double[] poseReading = timestampedDouble.value;
      if (queue[i].value.length < 10 || !allianceOptional.isPresent() || poseReading[7] == 0.0) {
        continue;
      }

      // [tx,ty,tz,r,p,y,latency,tagcount, max tag span in meters, average tag distance in meters,
      // average tag area]

      Pose3d poseEstimation =
          new Pose3d(
              new Translation3d(poseReading[0], poseReading[1], poseReading[2]),
              new Rotation3d(
                  Rotation2d.fromDegrees(poseReading[3]).getRadians(),
                  Rotation2d.fromDegrees(poseReading[4]).getRadians(),
                  Rotation2d.fromDegrees(poseReading[5])
                      .getRadians())); // Retrieves the pose estimation for the robot
      double latencyMS = poseReading[6];
      double tagCount = poseReading[7];
      double averageTagDistance = poseReading[9]; // Initializes the average tag distance to 0.0
      timestamp -= (latencyMS / 1e3); // Adjusts the timestamp by subtracting the latency in seconds

      poseEstimates.add(
          new PoseEstimate(
              poseEstimation,
              timestamp,
              averageTagDistance,
              tagCount)); // Creates a new PoseEstimate object and adds it to the poseEstimates
      // ArrayList
    }

    inputs.poseEstimates =
        poseEstimates; // Assigns the poseEstimates ArrayList to the inputs.poseEstimates variable
  }
}
