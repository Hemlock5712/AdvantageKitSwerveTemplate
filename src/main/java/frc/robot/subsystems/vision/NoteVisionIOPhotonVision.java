package frc.robot.subsystems.vision;

import edu.wpi.first.math.util.Units;
import org.photonvision.PhotonCamera;

public class NoteVisionIOPhotonVision implements NoteVisionIO {
  private final PhotonCamera camera;

  public NoteVisionIOPhotonVision(String cameraName) {
    camera = new PhotonCamera(cameraName);
  }

  @Override
  public void updateInputs(NoteVisionIOInputs inputs) {
    var result = camera.getLatestResult();

    var targets = result.getTargets();

    inputs.notePitches = new double[targets.size()];
    inputs.noteYaws = new double[targets.size()];

    for (int i = 0; i < targets.size(); i++) {
      inputs.noteYaws[i] = Units.degreesToRadians(targets.get(i).getYaw());
      inputs.notePitches[i] = Units.degreesToRadians(targets.get(i).getPitch());
    }
  }
}
