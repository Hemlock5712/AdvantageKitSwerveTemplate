// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.drive.Module;
import frc.robot.subsystems.vision.AprilTagVisionIO.AprilTagVisionIOInputs;
import frc.robot.util.FieldConstants;
import frc.robot.util.PoseEstimator.TimestampedVisionUpdate;
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
  private static final Pose3d[] cameraPoses;
  private static final double xyStdDevCoefficient;
  private static final double thetaStdDevCoefficient;

  private boolean enableVisionUpdates = true;

  private Consumer<List<TimestampedVisionUpdate>> visionConsumer = x -> {};
  private Map<Integer, Double> lastFrameTimes = new HashMap<>();
  private Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();

  private final AprilTagVisionIO[] io;
  private final AprilTagVisionIOInputs[] inputs;

  Pose2d robotPose = new Pose2d();

  static {
    if (Constants.currentMode == Mode.REAL || Constants.currentMode == Mode.REPLAY) {
      cameraPoses =
          new Pose3d[] {
            // Front left (forward facing, camera 1)
            new Pose3d(
                Units.inchesToMeters(2),
                Units.inchesToMeters(4.5),
                Units.inchesToMeters(33.5) + Module.getWheelRadius(),
                new Rotation3d(0.0, Units.degreesToRadians(0), 0.0)
                    .rotateBy(new Rotation3d(0.0, 0.0, 0))),
          };
      xyStdDevCoefficient = 0.01;
      thetaStdDevCoefficient = 0.01;
    } else {
      cameraPoses = new Pose3d[] {};
      xyStdDevCoefficient = 0.01;
      thetaStdDevCoefficient = 0.01;
    }
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
      Logger.processInputs("AprilTagVision/Inst" + Integer.toString(i), inputs[i]);
    }
    List<TimestampedVisionUpdate> visionUpdates = processVisionUpdates();
    logTagPoses();
    sendResultsToPoseEstimator(visionUpdates);
  }

  private List<TimestampedVisionUpdate> processVisionUpdates() {
    List<TimestampedVisionUpdate> visionUpdates = new ArrayList<>();
    for (int instanceIndex = 0; instanceIndex < io.length; instanceIndex++) {
      for (int frameIndex = 0;
          frameIndex < inputs[instanceIndex].getCaptureTimestamp().length;
          frameIndex++) {
        if (!shouldSkipFrame(instanceIndex)) {
          Pose3d cameraPose = cameraPoses[instanceIndex];
          var timestamp = inputs[instanceIndex].getCaptureTimestamp()[frameIndex];
          Pose3d robotPose3d = createRobotPose3d(instanceIndex, frameIndex);
          if (isRobotPoseOutOfBounds(robotPose3d)) {
            continue;
          }

          setRobotPose(robotPose3d.toPose2d());

          List<Pose3d> tagPoses = getTagPoses(instanceIndex);
          double avgDistance = calculateAverageDistance(tagPoses, cameraPose);

          double xyStdDev = xyStdDevCoefficient * Math.pow(avgDistance, 2.0) / tagPoses.size();
          double thetaStdDev =
              thetaStdDevCoefficient * Math.pow(avgDistance, 2.0) / tagPoses.size();
          visionUpdates.add(
              new TimestampedVisionUpdate(
                  timestamp, getRobotPose(), VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));
        }
      }
    }
    return visionUpdates;
  }

  private boolean shouldSkipFrame(int instanceIndex) {
    return inputs[instanceIndex].getCurrentTags().length < 1
        || inputs[instanceIndex].getEstimatedPose() == null
        || cameraPoses[instanceIndex] == null
        || !inputs[instanceIndex].isValid();
  }

  private Pose3d createRobotPose3d(int instanceIndex, int frameIndex) {
    return new Pose3d(
        new Translation3d(
            inputs[instanceIndex].getEstimatedPose()[frameIndex][0],
            inputs[instanceIndex].getEstimatedPose()[frameIndex][1],
            inputs[instanceIndex].getEstimatedPose()[frameIndex][2]),
        new Rotation3d(
            inputs[instanceIndex].getEstimatedPose()[frameIndex][3],
            inputs[instanceIndex].getEstimatedPose()[frameIndex][4],
            inputs[instanceIndex].getEstimatedPose()[frameIndex][5]));
  }

  private boolean isRobotPoseOutOfBounds(Pose3d robotPose3d) {
    return robotPose3d.getX() < -fieldBorderMargin
        || robotPose3d.getX() > FieldConstants.fieldLength + fieldBorderMargin
        || robotPose3d.getY() < -fieldBorderMargin
        || robotPose3d.getY() > FieldConstants.fieldWidth + fieldBorderMargin
        || robotPose3d.getZ() < -zMargin
        || robotPose3d.getZ() > zMargin;
  }

  private List<Pose3d> getTagPoses(int instanceIndex) {
    List<Pose3d> tagPoses = new ArrayList<>();
    Arrays.stream(inputs[instanceIndex].getCurrentTags())
        .forEachOrdered(
            tagId -> {
              lastTagDetectionTimes.put(tagId, Timer.getFPGATimestamp());
              Optional<Pose3d> tagPose = FieldConstants.aprilTags.getTagPose(tagId);
              tagPose.ifPresent(tagPoses::add);
            });
    return tagPoses;
  }

  private double calculateAverageDistance(List<Pose3d> tagPoses, Pose3d cameraPose) {
    // Calculate average distance to tag
    return tagPoses.stream()
        .mapToDouble(value -> value.getTranslation().getDistance(cameraPose.getTranslation()))
        .average()
        .orElse(0.0);
  }

  private void logTagPoses() {
    List<Pose3d> allTagPoses = new ArrayList<>();

    lastTagDetectionTimes.entrySet().stream()
        .filter(x -> Timer.getFPGATimestamp() - x.getValue() < targetLogTimeSecs)
        .forEach(x -> FieldConstants.aprilTags.getTagPose(x.getKey()).ifPresent(allTagPoses::add));

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
