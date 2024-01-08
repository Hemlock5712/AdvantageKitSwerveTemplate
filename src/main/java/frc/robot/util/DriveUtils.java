package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;

public class DriveUtils {
  public static Translation2d getCurrentDriveVector(double x, double y, double theta) {
    double magnitude = Math.hypot(x, y);
    double direction = Math.atan2(y, x);
    double newDirection = direction - theta;
    return new Translation2d(
        magnitude * Math.cos(newDirection), magnitude * Math.sin(newDirection));
  }

  public static double getDistanceFromVector(Translation2d vector, Translation2d point) {
    if (vector != null && point != null) {
      return ((vector.getX() * point.getY()) - (vector.getY() * point.getX())) / vector.getNorm();
    }
    return 0.0;
  }

  public static Translation2d translatePointToNewOrigin(Translation2d point, Translation2d space) {
    if (space == null) {
      return new Translation2d();
    }
    if (point == null) {
      return new Translation2d();
    }
    return new Translation2d(point.getX() - space.getX(), point.getY() - space.getY());
  }
}
