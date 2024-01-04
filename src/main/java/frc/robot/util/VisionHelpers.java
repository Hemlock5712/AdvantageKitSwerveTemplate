package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import java.util.Arrays;
import java.util.Objects;

public class VisionHelpers {
  public record PoseEstimate(
      Pose3d pose, double timestampSeconds, double averageTagDistance, int[] tagIDs) {
    @Override
    public boolean equals(Object obj) {
      if (this == obj) {
        return true;
      }
      if (obj == null || getClass() != obj.getClass()) {
        return false;
      }
      PoseEstimate other = (PoseEstimate) obj;
      return Arrays.equals(tagIDs, other.tagIDs)
          && Objects.equals(pose, other.pose)
          && Double.compare(timestampSeconds, other.timestampSeconds) == 0
          && Double.compare(averageTagDistance, other.averageTagDistance) == 0;
    }

    @Override
    public int hashCode() {
      return Objects.hash(
          Arrays.hashCode(getPose3dToArray(pose)),
          timestampSeconds,
          averageTagDistance,
          Arrays.hashCode(tagIDs));
    }

    @Override
    public String toString() {
      return "PoseEstimate{"
          + "pose="
          + pose.toString()
          + ", timestampSeconds="
          + Double.toString(timestampSeconds)
          + ", averageTagDistance="
          + Double.toString(averageTagDistance)
          + ", tagIDs="
          + Arrays.toString(tagIDs)
          + '}';
    }
  }

  public static double[] getPose3dToArray(Pose3d pose) {
    double[] result = new double[6];
    result[0] = pose.getTranslation().getX();
    result[1] = pose.getTranslation().getY();
    result[2] = pose.getTranslation().getZ();
    result[3] = Units.radiansToDegrees(pose.getRotation().getX());
    result[4] = Units.radiansToDegrees(pose.getRotation().getY());
    result[5] = Units.radiansToDegrees(pose.getRotation().getZ());
    return result;
  }
}
