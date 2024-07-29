// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.PoseEstimator.VisionObservation;
import frc.robot.subsystems.vision.AprilTagVisionIO.AprilTagVisionIOInputs;
import frc.robot.util.FieldConstants;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AprilTagVision extends SubsystemBase {

  // Margin around the field border
  private static final double fieldBorderMargin = 0.5;

  // Path for logging vision data
  private static final String VISION_PATH = "AprilTagVision/Inst";

  private boolean enableVisionUpdates = true;

  private Consumer<List<VisionObservation>> visionConsumer = x -> {};
  private Map<Integer, Double> lastFrameTimes = new HashMap<>();
  private Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();

  private final AprilTagVisionIO[] io;
  private final AprilTagVisionIOInputs[] inputs;

  public void setDataInterfaces(Consumer<List<VisionObservation>> visionConsumer) {
    this.visionConsumer = visionConsumer;
  }

  @AutoLogOutput(key = "/AprilTagVision/poseEstimateCount")
  public int getPoseEstimationCount() {
    int count = 0;
    for (AprilTagVisionIOInputs input : inputs) {
      count += input.poseEstimates.size();
    }
    return count;
  }

  public AprilTagVision(AprilTagVisionIO... io) {
    System.out.println("[Init] Creating AprilTagVision");
    this.io = io;
    inputs = new AprilTagVisionIOInputs[io.length];
    for (int i = 0; i < io.length; i++) {
      inputs[i] = new AprilTagVisionIOInputs();
    }

    // Create map of last frame times for instances
    for (int i = 0; i < io.length; i++) {
      lastFrameTimes.put(i, 0.0);
    }

    // Create map of last detection times for tags
    FieldConstants.aprilTags.getTags().forEach(tag -> lastTagDetectionTimes.put(tag.ID, 0.0));
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs(VISION_PATH + Integer.toString(i), inputs[i]);
    }
    List<VisionObservation> visionUpdates = processPoseEstimates();
    sendResultsToPoseEstimator(visionUpdates);
  }

  /**
   * Process the pose estimates and generate vision updates.
   *
   * @return List of timestamped vision updates
   */
  private List<VisionObservation> processPoseEstimates() {
    List<VisionObservation> visionUpdates = new ArrayList<>();
    for (int instanceIndex = 0; instanceIndex < io.length; instanceIndex++) {
      for (PoseEstimate poseEstimates : inputs[instanceIndex].poseEstimates) {
        if (shouldSkipPoseEstimate(poseEstimates)) {
          continue;
        }
        double timestamp = poseEstimates.timestampSeconds;
        Pose2d robotPose = poseEstimates.pose;
        double xyStdDev = calculateXYStdDev(poseEstimates, poseEstimates.tagCount);
        double thetaStdDev = 9999999;
        if (DriverStation.isDisabled()) {
          thetaStdDev = calculateThetaStdDev(poseEstimates, poseEstimates.tagCount);
        }
        // double thetaStdDev = calculateThetaStdDev(poseEstimates, poseEstimates.tagCount());
        visionUpdates.add(
            new VisionObservation(
                robotPose, timestamp, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));
      }
    }
    return visionUpdates;
  }

  /**
   * Check if the pose estimate should be skipped.
   *
   * @param poseEstimates The pose estimate
   * @return True if the pose estimate should be skipped, false otherwise
   */
  private boolean shouldSkipPoseEstimate(PoseEstimate poseEstimates) {
    return poseEstimates.tagCount < 1
        || poseEstimates.pose == null
        || isOutsideFieldBorder(poseEstimates.pose);
  }

  /**
   * Check if the robot pose is outside the field border.
   *
   * @param robotPose The robot pose
   * @return True if the robot pose is outside the field border, false otherwise
   */
  private boolean isOutsideFieldBorder(Pose2d robotPose) {
    return robotPose.getX() < -fieldBorderMargin
        || robotPose.getX() > FieldConstants.fieldLength + fieldBorderMargin
        || robotPose.getY() < -fieldBorderMargin
        || robotPose.getY() > FieldConstants.fieldWidth + fieldBorderMargin;
  }

  /**
   * Calculate the standard deviation of the x and y coordinates.
   *
   * @param poseEstimates The pose estimate
   * @param tagPosesSize The number of detected tag poses
   * @return The standard deviation of the x and y coordinates
   */
  private double calculateXYStdDev(PoseEstimate poseEstimates, int tagPosesSize) {
    return xyStdDevCoefficient * Math.pow(poseEstimates.avgTagDist, 2.0) / tagPosesSize;
  }

  /**
   * Calculate the standard deviation of the theta coordinate.
   *
   * @param poseEstimates The pose estimate
   * @param tagPosesSize The number of detected tag poses
   * @return The standard deviation of the theta coordinate
   */
  private double calculateThetaStdDev(PoseEstimate poseEstimates, int tagPosesSize) {
    return thetaStdDevCoefficient * Math.pow(poseEstimates.avgTagDist, 2.0) / tagPosesSize;
  }

  /**
   * Send the vision updates to the pose estimator.
   *
   * @param visionUpdates The list of vision updates
   */
  private void sendResultsToPoseEstimator(List<VisionObservation> visionUpdates) {
    if (enableVisionUpdates) {
      visionConsumer.accept(visionUpdates);
    }
  }

  /**
   * Set whether to enable vision updates.
   *
   * @param enableVisionUpdates True to enable vision updates, false otherwise
   */
  public void setEnableVisionUpdates(boolean enableVisionUpdates) {
    this.enableVisionUpdates = enableVisionUpdates;
  }
}
