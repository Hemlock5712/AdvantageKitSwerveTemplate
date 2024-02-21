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

/** This class represents the implementation of AprilTagVisionIO using Limelight camera. */
public class AprilTagVisionIOLimelight implements AprilTagVisionIO {

  String limelightName;
  private final StringSubscriber observationSubscriber;

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
            .getStringTopic("json")
            .subscribe("", PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
  }

  /**
   * Updates the inputs for AprilTag vision.
   *
   * @param inputs The AprilTagVisionIOInputs object containing the inputs.
   */
  @Override
  public void updateInputs(AprilTagVisionIOInputs inputs) {
    TimestampedString[] queue =
        observationSubscriber.readQueue(); // Reads the queue of timestamped strings
    ArrayList<PoseEstimate> poseEstimates =
        new ArrayList<>(); // Creates an empty ArrayList to store pose estimates

    // Iterates over each timestamped string in the queue
    for (int i = 0; i < Math.min(queue.length, 3); i++) {
      TimestampedString timestampedString = queue[i];
      double timestamp = timestampedString.timestamp / 1e6; // Converts the timestamp to seconds
      LimelightHelpers.Results results =
          LimelightHelpers.parseJsonDump(timestampedString.value)
              .targetingResults; // Parses the JSON dump and retrieves the targeting results
      Optional<Alliance> allianceOptional =
          DriverStation.getAlliance(); // Retrieves the alliance information from the DriverStation

      // Checks if there are no targets or if the alliance information is not present
      if (results.targets_Fiducials.length == 0 || !allianceOptional.isPresent()) {
        continue; // Skips to the next iteration of the loop
      }

      double latencyMS =
          results.latency_capture
              + results.latency_pipeline; // Calculates the total latency in milliseconds
      Pose3d poseEstimation =
          results.getBotPose3d_wpiBlue(); // Retrieves the pose estimation for the robot
      double averageTagDistance = 0.0; // Initializes the average tag distance to 0.0
      timestamp -= (latencyMS / 1e3); // Adjusts the timestamp by subtracting the latency in seconds

      int[] tagIDs =
          new int[results.targets_Fiducials.length]; // Creates an array to store the tag IDs

      // Iterates over each target in the targeting results
      for (int aprilTags = 0; aprilTags < results.targets_Fiducials.length; aprilTags++) {
        tagIDs[aprilTags] =
            (int)
                results.targets_Fiducials[aprilTags].fiducialID; // Retrieves and stores the tag ID
        averageTagDistance +=
            results
                .targets_Fiducials[aprilTags]
                .getTargetPose_CameraSpace()
                .getTranslation()
                .getNorm(); // Calculates the sum of the tag distances
      }

      averageTagDistance /= tagIDs.length; // Calculates the average tag distance
      poseEstimates.add(
          new PoseEstimate(
              poseEstimation,
              timestamp,
              averageTagDistance,
              tagIDs)); // Creates a new PoseEstimate object and adds it to the poseEstimates
      // ArrayList
    }

    inputs.poseEstimates =
        poseEstimates; // Assigns the poseEstimates ArrayList to the inputs.poseEstimates variable
  }
}
