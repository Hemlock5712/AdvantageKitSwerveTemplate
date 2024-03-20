package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.ArrayList;
import java.util.Optional;

public class PoseLog {
  private final ArrayList<Double> timestamps = new ArrayList<>();
  private final ArrayList<Pose2d> poses = new ArrayList<>();
  private final double secondsToRetain = 0.2;

  public PoseLog() {}

  /**
   * this removes all recorded poses before the time provided
   *
   * @param timeInSeconds
   * @return what the pose is interpolated to be at the provided time
   */
  public Pose2d getPoseAtTime(double timeInSeconds) {
    if (poses.isEmpty()) {
      return null;
    }

    Optional<Integer> timestampIndex = binarySearchClosestTimeIndex(timeInSeconds);

    if (timestampIndex.isEmpty()) {
      return poses.get(0);
    }

    // the time provided is after all our data, return most recent pose
    if (timestampIndex.get() == timestamps.size() - 1) {
      return poses.get(timestampIndex.get());
    }

    Pose2d poseBeforeTime = poses.get(timestampIndex.get());
    double prevTimestamp = timestamps.get(timestampIndex.get());
    Pose2d poseAfterTime = poses.get(timestampIndex.get() + 1);
    double afterTimestamp = timestamps.get(timestampIndex.get() + 1);

    Pose2d output =
        poseBeforeTime.interpolate(
            poseAfterTime, (timeInSeconds - prevTimestamp) / (afterTimestamp - prevTimestamp));

    for (int i = 0; i < timestampIndex.get(); i++) {
      timestamps.remove(0);
      poses.remove(0);
    }

    return output;
  }

  /**
   * @param timeInSeconds
   * @return the index of the time before the time provided
   */
  private Optional<Integer> binarySearchClosestTimeIndex(double timeInSeconds) {
    int upper = timestamps.size() - 1;
    int lower = 0;

    while (lower < upper) {
      int middle = lower + (upper - lower) / 2;

      // the time is after the end of our timestamps, return the last index
      if (timestamps.size() - 1 == middle) {
        return Optional.of(middle);
      }

      boolean isTooHigh = timestamps.get(middle) > timeInSeconds;
      boolean isTooLow = timestamps.get(middle + 1) < timeInSeconds;

      if (isTooHigh) {
        upper = middle;
        continue;
      }

      if (isTooLow) {
        lower = middle + 1;
        continue;
      }

      return Optional.of(middle);
    }

    return Optional.empty();
  }

  public void addNewPose(Pose2d pose, double timeInSeconds) {
    if (!timestamps.isEmpty() && timeInSeconds <= timestamps.get(timestamps.size() - 1)) {
      return;
    }

    timestamps.add(timeInSeconds);
    poses.add(pose);

    while (!timestamps.isEmpty() && timestamps.get(0) < timeInSeconds - secondsToRetain) {
      timestamps.remove(0);
      poses.remove(0);
    }
  }
}
