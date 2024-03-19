package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.FieldConstants;
import java.util.stream.Stream;

public class AutoConstants {
  public static final double DISTANCE_TO_TRUST_CAMERA = 3;
  public static final double DRIVE_TO_PICKUP_INTERRUPT_DISTANCE = 0.8;
  public static final double SHOOTING_DISTANCE_OFFSET_TOLERANCE = 0.5;
  public static final double SHOOTING_ANGLE_OFFSET_TOLERANCE = Units.degreesToRadians(5);
  public static final Translation2d[] AUTO_NOTES =
      Stream.concat(
              Stream.of(FieldConstants.StagingLocations.spikeTranslations),
              Stream.of(FieldConstants.StagingLocations.centerlineTranslations))
          .toArray(Translation2d[]::new);

  public static class AutoNoteOffsetThresholds {
    public static final double WHILE_ROUTING = 0.5;
    public static final double WHILE_ATTEMPTING_PICKUP = 1;
  }

  public static class ShootingTranslations {
    public static final Translation2d A = new Translation2d(1, 6.7);
    public static final Translation2d B =
        new Translation2d(1.4, FieldConstants.Speaker.centerSpeakerOpening.getY());
    public static final Translation2d C = new Translation2d(0.9, 4.3);
    public static final Translation2d D = new Translation2d(3.7, 5.7);
    public static final Translation2d E = new Translation2d(4.8, 6.2);
    public static final Translation2d F = new Translation2d(3.1, 2.9);
    public static final Translation2d G = new Translation2d(4.3, 2.3);
  }

  public static class NotePickupLocations {
    public static final Pose2d X =
        new Pose2d(new Translation2d(6.3, 6.5), Rotation2d.fromDegrees(-10));
    public static final Pose2d Y =
        new Pose2d(
            new Translation2d(6.3, FieldConstants.fieldWidth / 2), Rotation2d.fromDegrees(0));
    public static final Pose2d Z =
        new Pose2d(new Translation2d(6.3, 1.7), Rotation2d.fromDegrees(10));
  }
}
