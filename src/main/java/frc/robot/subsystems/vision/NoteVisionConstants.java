package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class NoteVisionConstants {
  public record CameraConfig(Transform3d cameraPose, double minDistance, String name) {
    public NoteVisionIOPhotonVision makePhotonVision() {
      return new NoteVisionIOPhotonVision(name);
    }
  }

  public static final CameraConfig[] CAMERA_CONFIGS = {
    // center
    new CameraConfig(
        new Transform3d(
            new Translation3d(0.4, 0, 0.45), new Rotation3d(0, Units.degreesToRadians(20), 0)),
        0.7,
        "lefty" // todo fix
        ),
    // center
    new CameraConfig(
        // left TODO: find measure position
        new Transform3d(
            new Translation3d(0.15, 0.2, 0.45),
            new Rotation3d(0, Units.degreesToRadians(20), Units.degreesToRadians(60))),
        0.7, // todo measure
        "left"),
    new CameraConfig(
        // right TODO: find measure position
        new Transform3d(
            new Translation3d(-0.15, 0.2, 0.45),
            new Rotation3d(0, Units.degreesToRadians(20), Units.degreesToRadians(-60))),
        0.7, // todo measure
        "right"),
  };
  public static final double LIFECAM_3000_HFOV = 55;
  public static final double LIFECAM_3000_VFOV = 35;

  public static final double NOTE_GROUPING_TOLERANCE = 0.5;

  public static final double MIN_CAMERA_DISTANCE = 0.7;
  public static final double MAX_CAMERA_DISTANCE = 5;

  public static final double NOTE_EXPIRATION = 2;
  public static final double IN_CAMERA_EXPIRATION = 0.1;
  public static final double MAX_ARM_POS_RAD = Units.degreesToRadians(3);
}
