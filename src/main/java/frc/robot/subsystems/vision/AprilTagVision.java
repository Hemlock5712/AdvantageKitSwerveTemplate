// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.AprilTagVisionIO.AprilTagVisionIOInputs;
import frc.robot.util.FieldConstants;
import frc.robot.util.PoseEstimator.TimestampedVisionUpdate;
import frc.robot.util.VisionHelpers.PoseEstimate;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Consumer;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

@Getter
@Setter
public class AprilTagVision extends SubsystemBase {
  private static final double targetLogTimeSecs = 0.1;
  private static final double fieldBorderMargin = 0.5;
  private static final double zMargin = 0.75;
  private static final double xyStdDevCoefficient;
  private static final double thetaStdDevCoefficient;
  private static final String VISION_PATH = "AprilTagVision/Inst";

  private boolean enableVisionUpdates = true;

  private Consumer<List<TimestampedVisionUpdate>> visionConsumer = x -> {};
  private Map<Integer, Double> lastFrameTimes = new HashMap<>();
  private Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();

  private final AprilTagVisionIO[] io;
  private final AprilTagVisionIOInputs[] inputs;

  Pose2d robotPose = new Pose2d();

  static {
    xyStdDevCoefficient = 0.01;
    thetaStdDevCoefficient = 0.01;
  }

  public void setDataInterfaces(Consumer<List<TimestampedVisionUpdate>> visionConsumer) {
    this.visionConsumer = visionConsumer;
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
    List<TimestampedVisionUpdate> visionUpdates = processPoseEstimates();
    sendResultsToPoseEstimator(visionUpdates);
  }

  private List<TimestampedVisionUpdate> processPoseEstimates() {
    List<TimestampedVisionUpdate> visionUpdates = new ArrayList<>();
    for (int instanceIndex = 0; instanceIndex < io.length; instanceIndex++) {
      for (PoseEstimate poseEstimates : inputs[instanceIndex].poseEstimates) {
        if (shouldSkipPoseEstimate(poseEstimates)) {
          continue;
        }
        double timestamp = poseEstimates.timestampSeconds();
        Pose3d robotPose3d = poseEstimates.pose();
        setRobotPose(robotPose3d.toPose2d());
        List<Pose3d> tagPoses = getTagPoses(poseEstimates);
        double xyStdDev = calculateXYStdDev(poseEstimates, tagPoses.size());
        double thetaStdDev = calculateThetaStdDev(poseEstimates, tagPoses.size());
        visionUpdates.add(
            new TimestampedVisionUpdate(
                timestamp, getRobotPose(), VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));
        logData(instanceIndex, timestamp, robotPose3d, tagPoses);
      }
    }
    return visionUpdates;
  }

  private boolean shouldSkipPoseEstimate(PoseEstimate poseEstimates) {
    return poseEstimates.tagIDs().length < 1
        || poseEstimates.pose() == null
        || isOutsideFieldBorder(poseEstimates.pose());
  }

  private boolean isOutsideFieldBorder(Pose3d robotPose3d) {
    return robotPose3d.getX() < -fieldBorderMargin
        || robotPose3d.getX() > FieldConstants.fieldLength + fieldBorderMargin
        || robotPose3d.getY() < -fieldBorderMargin
        || robotPose3d.getY() > FieldConstants.fieldWidth + fieldBorderMargin
        || robotPose3d.getZ() < -zMargin
        || robotPose3d.getZ() > zMargin;
  }

  private List<Pose3d> getTagPoses(PoseEstimate poseEstimates) {
    List<Pose3d> tagPoses = new ArrayList<>();
    Arrays.stream(poseEstimates.tagIDs())
        .forEachOrdered(
            tagId -> {
              lastTagDetectionTimes.put(tagId, Timer.getFPGATimestamp());
              Optional<Pose3d> tagPose = FieldConstants.aprilTags.getTagPose(tagId);
              tagPose.ifPresent(tagPoses::add);
            });
    return tagPoses;
  }

  private double calculateXYStdDev(PoseEstimate poseEstimates, int tagPosesSize) {
    return xyStdDevCoefficient * Math.pow(poseEstimates.averageTagDistance(), 2.0) / tagPosesSize;
  }

  private double calculateThetaStdDev(PoseEstimate poseEstimates, int tagPosesSize) {
    return thetaStdDevCoefficient
        * Math.pow(poseEstimates.averageTagDistance(), 2.0)
        / tagPosesSize;
  }

  private void logData(
      int instanceIndex, double timestamp, Pose3d robotPose3d, List<Pose3d> tagPoses) {
    Logger.recordOutput(
        VISION_PATH + Integer.toString(instanceIndex) + "/LatencySecs",
        Timer.getFPGATimestamp() - timestamp);
    Logger.recordOutput(
        VISION_PATH + Integer.toString(instanceIndex) + "/RobotPose", getRobotPose());
    Logger.recordOutput(
        VISION_PATH + Integer.toString(instanceIndex) + "/RobotPose3d", robotPose3d);
    Logger.recordOutput(
        VISION_PATH + Integer.toString(instanceIndex) + "/TagPoses",
        tagPoses.toArray(new Pose3d[tagPoses.size()]));
    logTagPoses();
  }

  private void logTagPoses() {
    List<Pose3d> allTagPoses = new ArrayList<>();
    for (Map.Entry<Integer, Double> detectionEntry : lastTagDetectionTimes.entrySet()) {
      if (Timer.getFPGATimestamp() - detectionEntry.getValue() < targetLogTimeSecs) {
        Optional<Pose3d> tagPose = FieldConstants.aprilTags.getTagPose(detectionEntry.getKey());
        if (tagPose.isPresent()) {
          allTagPoses.add(tagPose.get());
        }
      }
    }
    Logger.recordOutput(
        "AprilTagVision/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
  }

  private void sendResultsToPoseEstimator(List<TimestampedVisionUpdate> visionUpdates) {
    if (enableVisionUpdates) {
      visionConsumer.accept(visionUpdates);
    }
  }

  public Pose2d getRobotPose() {
    return robotPose;
  }
}
