// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Module;
import frc.robot.subsystems.vision.AprilTagVisionIO.AprilTagVisionIOInputs;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import frc.robot.util.FieldConstants;
import frc.robot.util.PoseEstimator.TimestampedVisionUpdate;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

public class AprilTagVision extends SubsystemBase {
  private static final double targetLogTimeSecs = 0.1;
  private static final double fieldBorderMargin = 0.5;
  private static final double zMargin = 0.75;
  private static final Pose3d[] cameraPoses;
  private static final double xyStdDevCoefficient;
  private static final double thetaStdDevCoefficient;

  private boolean enableVisionUpdates = true;
  private Alert enableVisionUpdatesAlert =
      new Alert("Vision updates are temporarily disabled.", AlertType.WARNING);
  private Consumer<List<TimestampedVisionUpdate>> visionConsumer = (x) -> {};
  private Map<Integer, Double> lastFrameTimes = new HashMap<>();
  private Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();

  private final AprilTagVisionIO[] io;
  private final AprilTagVisionIOInputs[] inputs;

  Pose2d robotPose = new Pose2d();

  static {
    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
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
        break;
      default:
        cameraPoses = new Pose3d[] {};
        xyStdDevCoefficient = 0.01;
        thetaStdDevCoefficient = 0.01;
        break;
    }
  }

  /** Sets whether vision updates for odometry are enabled. */
  public void setVisionUpdatesEnabled(boolean enabled) {
    enableVisionUpdates = enabled;
    enableVisionUpdatesAlert.set(!enabled);
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
    FieldConstants.aprilTags
        .getTags()
        .forEach(
            (AprilTag tag) -> {
              lastTagDetectionTimes.put(tag.ID, 0.0);
            });
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
          frameIndex < inputs[instanceIndex].captureTimestamp.length;
          frameIndex++) {
        if (!shouldSkipFrame(instanceIndex)) {
          Pose3d cameraPose = cameraPoses[instanceIndex];
          var timestamp = inputs[instanceIndex].captureTimestamp[frameIndex];
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
                  timestamp, getPose2d(), VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));
        }
      }
    }
    return visionUpdates;
  }

  private boolean shouldSkipFrame(int instanceIndex) {
    return inputs[instanceIndex].currentTags.length < 1
        || inputs[instanceIndex].estimatedPose == null
        || cameraPoses[instanceIndex] == null
        || !inputs[instanceIndex].valid;
  }

  private Pose3d createRobotPose3d(int instanceIndex, int frameIndex) {
    return new Pose3d(
        new Translation3d(
            inputs[instanceIndex].estimatedPose[frameIndex][0],
            inputs[instanceIndex].estimatedPose[frameIndex][1],
            inputs[instanceIndex].estimatedPose[frameIndex][2]),
        new Rotation3d(
            inputs[instanceIndex].estimatedPose[frameIndex][3],
            inputs[instanceIndex].estimatedPose[frameIndex][4],
            inputs[instanceIndex].estimatedPose[frameIndex][5]));
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
    for (int tagCounter = 0; tagCounter < inputs[instanceIndex].currentTags.length; tagCounter++) {
      int tagId = inputs[instanceIndex].currentTags[tagCounter];
      lastTagDetectionTimes.put(tagId, Timer.getFPGATimestamp());
      Optional<Pose3d> tagPose = FieldConstants.aprilTags.getTagPose(tagId);
      if (tagPose.isPresent()) {
        tagPoses.add(tagPose.get());
      }
    }
    return tagPoses;
  }

  private double calculateAverageDistance(List<Pose3d> tagPoses, Pose3d cameraPose) {
    double totalDistance = 0.0;
    for (Pose3d tagPose : tagPoses) {
      totalDistance += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
    }
    return totalDistance / tagPoses.size();
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

  public void setRobotPose(Pose2d robotPose) {
    this.robotPose = robotPose;
  }

  public Pose2d getPose2d() {
    return this.robotPose;
  }
}
