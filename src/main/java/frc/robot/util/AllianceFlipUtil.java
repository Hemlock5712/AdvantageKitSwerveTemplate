package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

/** Utility functions for flipping from the blue to red alliance. */
public class AllianceFlipUtil {
  /**
   * Flips an x coordinate to the correct side of the field based on the current alliance color.
   *
   * @param xCoordinate The x coordinate to be flipped.
   * @return The flipped x coordinate.
   */
  public static double apply(double xCoordinate) {
    if (shouldFlip()) {
      return FieldConstants.fieldLength - xCoordinate;
    } else {
      return xCoordinate;
    }
  }

  /**
   * Flips a translation to the correct side of the field based on the current alliance color.
   *
   * @param translation The translation to be flipped.
   * @return The flipped translation.
   */
  public static Translation2d apply(Translation2d translation) {
    if (shouldFlip()) {
      return new Translation2d(apply(translation.getX()), translation.getY());
    } else {
      return translation;
    }
  }

  /**
   * Flips a rotation based on the current alliance color.
   *
   * @param rotation The rotation to be flipped.
   * @return The flipped rotation.
   */
  public static Rotation2d apply(Rotation2d rotation) {
    if (shouldFlip()) {
      return new Rotation2d(-rotation.getCos(), rotation.getSin());
    } else {
      return rotation;
    }
  }

  /**
   * Flips a pose to the correct side of the field based on the current alliance color.
   *
   * @param pose The pose to be flipped.
   * @return The flipped pose.
   */
  public static Pose2d apply(Pose2d pose) {
    if (shouldFlip()) {
      return new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()));
    } else {
      return pose;
    }
  }

  /**
   * Checks if the alliance color should be flipped.
   *
   * @return True if the alliance color should be flipped, false otherwise.
   */
  public static boolean shouldFlip() {
    Optional<Alliance> optional = DriverStation.getAlliance();
    return optional.isPresent() && optional.get() == Alliance.Red;
  }
}
