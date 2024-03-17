package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.FieldConstants;
import java.util.Arrays;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

public class NoteVisionIOSim implements NoteVisionIO {
  private final VisionSystemSim visionSim;
  private final TargetModel targetModel = new TargetModel(0.2, 0.2, 0.05);
  private final PhotonCamera camera = new PhotonCamera("center note camera");
  private final PhotonCameraSim cameraSim;

  private final Pose3d[] notePoses = new Pose3d[8];

  public NoteVisionIOSim(VisionSystemSim visionSim) {
    this.visionSim = visionSim;

    for (int i = 0; i < 3; i++) {
      var translation = FieldConstants.StagingLocations.spikeTranslations[i];
      notePoses[i] = new Pose3d(translation.getX(), translation.getY(), 0, new Rotation3d());
    }

    for (int i = 0; i < 5; i++) {
      var translation = FieldConstants.StagingLocations.centerlineTranslations[i];
      notePoses[i + 3] = new Pose3d(translation.getX(), translation.getY(), 0, new Rotation3d());
    }

    visionSim.addVisionTargets(
        Arrays.stream(notePoses)
            .map(notePose -> new VisionTargetSim(notePose, targetModel))
            .toArray(VisionTargetSim[]::new));

    final var cameraProps = new SimCameraProperties();
    cameraProps.setAvgLatencyMs(10);
    cameraProps.setFPS(30);
    cameraProps.setCalibration(1200, 960, Rotation2d.fromDegrees(70));
    cameraSim = new PhotonCameraSim(camera, cameraProps);
    visionSim.addCamera(cameraSim, NoteVisionConstants.CAMERA_POS);
  }

  @Override
  public void updateInputs(NoteVisionIOInputs inputs) {
    var result = camera.getLatestResult();

    var targets = result.getTargets();

    inputs.notePitches = new double[targets.size()];
    inputs.noteYaws = new double[targets.size()];
    inputs.timeStampSeconds = result.getTimestampSeconds();

    for (int i = 0; i < targets.size(); i++) {
      inputs.noteYaws[i] = -Units.degreesToRadians(targets.get(i).getYaw());
      inputs.notePitches[i] = -Units.degreesToRadians(targets.get(i).getPitch());
    }
  }
}
