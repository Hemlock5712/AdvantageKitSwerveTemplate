package frc.robot.subsystems.gamepiece;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.FieldConstants;
import frc.robot.util.VisionHelpers.GamePiece;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class GamePieceVisionIOPhotonVisionSIM implements GamePieceVisionIO {
  private final PhotonCamera camera;
  private VisionSystemSim visionSim;
  private PhotonCameraSim cameraSim;
  private final Supplier<Pose2d> poseSupplier;

  public GamePieceVisionIOPhotonVisionSIM(
      String identifier, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;
    camera = new PhotonCamera(identifier);
    visionSim = new VisionSystemSim("gamePieceVision");
    visionSim.update(poseSupplier.get());
    var cameraProp = new SimCameraProperties();
    cameraSim = new PhotonCameraSim(camera, cameraProp);
    visionSim.addCamera(cameraSim, robotToCamera);
    cameraSim.prop.setCalibration(640, 480, Rotation2d.fromDegrees(80));
    cameraSim.setMaxSightRange(10);
    cameraSim.setMinTargetAreaPixels(1.0);
    Translation2d[] centerGamePieces = FieldConstants.StagingLocations.centerlineTranslations;
    for (Translation2d centerGamePiece : centerGamePieces) {
      Pose3d targetPose =
          new Pose3d(
              centerGamePiece.getX(),
              centerGamePiece.getY(),
              Units.inchesToMeters(1),
              new Rotation3d());
      TargetModel targetModel =
          new TargetModel(
              drawGamePieceVision(Units.inchesToMeters(7), Units.inchesToMeters(5), 10, 10));
      // The given target model at the given pose
      VisionTargetSim visionTarget = new VisionTargetSim(targetPose, targetModel);
      visionSim.addVisionTargets(visionTarget);
    }
  }

  @Override
  public void updateInputs(GamePieceVisionIOInputs inputs) {
    ArrayList<GamePiece> gamePieces = new ArrayList<>();
    visionSim.update(poseSupplier.get());
    List<PhotonTrackedTarget> currentTargets = cameraSim.getCamera().getLatestResult().getTargets();
    for (PhotonTrackedTarget target : currentTargets) {
      var detectedCorners = target.getDetectedCorners();

      double highestX = 0;
      double highestY = 0;
      double lowestX = 9999999;
      double lowestY = 9999999;
      for (TargetCorner corner : detectedCorners) {

        if (corner.x > highestX) {
          highestX = corner.x;
        }
        if (corner.y > highestY) {
          highestY = corner.y;
        }
        if (corner.x < lowestX) {
          lowestX = corner.x;
        }
        if (corner.y < lowestY) {
          lowestY = corner.y;
        }
      }

      gamePieces.add(new GamePiece(highestX - lowestX, highestY - lowestY));
    }
    inputs.gamePieces = gamePieces;
  }

  public List<Translation3d> drawGamePieceVision(
      double majorRadius, double minorRadius, int numMajorDivisions, int numMinorDivisions) {
    List<Translation3d> vertices = new ArrayList<>();
    double majorAngleIncrement = 2 * Math.PI / numMajorDivisions;
    double minorAngleIncrement = 2 * Math.PI / numMinorDivisions;

    for (int i = 0; i < numMajorDivisions; i++) {
      double majorAngle = i * majorAngleIncrement;
      for (int j = 0; j < numMinorDivisions; j++) {
        double minorAngle = j * minorAngleIncrement;
        double x = (majorRadius + minorRadius * Math.cos(minorAngle)) * Math.cos(majorAngle);
        double y = (majorRadius + minorRadius * Math.cos(minorAngle)) * Math.sin(majorAngle);
        double z = minorRadius * Math.sin(minorAngle);
        vertices.add(new Translation3d(x, y, z));
      }
    }
    return vertices;
  }
}
