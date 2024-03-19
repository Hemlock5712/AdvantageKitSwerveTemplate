package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.robot.commands.auto.AutoConstants;
import frc.robot.util.AllianceFlipUtil;
import java.util.ArrayList;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;
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
  private Pose3d[] notePoses = new Pose3d[8];

  public NoteVisionIOSim(VisionSystemSim visionSim) {
    this.visionSim = visionSim;
    resetNotePoses();

    final var cameraProps = new SimCameraProperties();
    cameraProps.setAvgLatencyMs(10);
    cameraProps.setFPS(30);
    cameraProps.setCalibration(1200, 960, Rotation2d.fromDegrees(70));
    visionSim.addCamera(new PhotonCameraSim(camera, cameraProps), NoteVisionConstants.CAMERA_POS);
  }

  public void resetNotePoses() {
    notePoses =
        Arrays.stream(AutoConstants.AUTO_NOTES)
            .map(AllianceFlipUtil::apply)
            .map(
                translation ->
                    new Pose3d(translation.getX() - 1, translation.getY(), 0, new Rotation3d()))
            .toArray(Pose3d[]::new);
    updateNoteTargets();
  }

  private void updateNoteTargets() {
    visionSim.clearVisionTargets();

    visionSim.addVisionTargets(
        Arrays.stream(notePoses)
            .map(notePose -> new VisionTargetSim(notePose, targetModel))
            .toArray(VisionTargetSim[]::new));

    Logger.recordOutput("note vision sim locations", notePoses);
  }

  @Override
  public void updateInputs(NoteVisionIOInputs inputs) {
    var result = camera.getLatestResult();

    var targets = result.getTargets();

    inputs.notePitches = new double[targets.size()];
    inputs.noteYaws = new double[targets.size()];
    inputs.timeStampSeconds = result.getTimestampSeconds();

    for (int i = 0; i < targets.size(); i++) {
      inputs.noteYaws[i] =
          -Units.degreesToRadians(targets.get(i).getYaw() + 5 * (2 * Math.random() - 1));
      inputs.notePitches[i] =
          -Units.degreesToRadians(targets.get(i).getPitch() + 5 * (2 * Math.random() - 1));
    }
  }

  public Translation2d[] getNoteLocations() {
    return Arrays.stream(notePoses)
        .map(Pose3d::toPose2d)
        .map(Pose2d::getTranslation)
        .toArray(Translation2d[]::new);
  }

  public void removeNote(int index) {
    var noteArrayList = new ArrayList<>(Arrays.asList(notePoses));
    noteArrayList.remove(index);
    notePoses = noteArrayList.toArray(new Pose3d[0]);
    updateNoteTargets();
  }
}
