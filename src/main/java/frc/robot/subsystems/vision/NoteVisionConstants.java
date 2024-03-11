package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class NoteVisionConstants {
  public static final Pose3d CAMERA_POS =
      new Pose3d(
          new Translation3d(0.4, 0, 0.45), new Rotation3d(0, -Units.degreesToRadians(30), 0));
  public static final double LIFECAM_3000_HFOV = 55;
  public static final double LIFECAM_3000_VFOV = 35;

  public static final double NOTE_GROUPING_TOLERANCE = 0.5;

  public static final double MIN_CAMERA_DISTANCE = 0;
  public static final double MAX_CAMERA_DISTANCE = 5;

  public static final double OUT_OF_CAMERA_EXPIRATION = 5;
}
