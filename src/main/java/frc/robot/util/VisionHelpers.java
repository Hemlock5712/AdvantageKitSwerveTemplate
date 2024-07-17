package frc.robot.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/**
 * The VisionHelpers class provides utility methods and record classes for vision-related
 * operations.
 */
public class VisionHelpers {

  public record GamePiece(double width, double hight) {}

  /**
   * Converts a Pose3d object to an array of doubles.
   *
   * @param pose The Pose3d object to convert.
   * @return The array of doubles representing the pose.
   */
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

  /**
   * Represents a timestamped vision update with pose and standard deviations.
   *
   * @param timestamp The timestamp of the vision update.
   * @param pose The pose estimate.
   * @param stdDevs The standard deviations matrix.
   */
  public record TimestampedVisionUpdate(
      /** The timestamp of the vision update. */
      double timestamp,
      /** The pose estimate. */
      Pose2d pose,
      /** The standard deviations matrix. */
      Matrix<N3, N1> stdDevs) {

    /**
     * Returns a string representation of this vision update.
     *
     * @return The string representation.
     */
    @Override
    public String toString() {
      return "VisionUpdate{"
          + "timestamp="
          + Double.toString(timestamp)
          + ", pose="
          + pose.toString()
          + ", stdDevs="
          + stdDevs.toString()
          + '}';
    }
  }
}
