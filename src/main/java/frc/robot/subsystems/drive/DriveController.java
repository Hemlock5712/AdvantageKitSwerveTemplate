package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import java.util.Optional;
import java.util.function.Supplier;

public class DriveController {
  private Optional<Supplier<Rotation2d>> headingSupplier = Optional.empty();
  private DriveModeType driveModeType = DriveModeType.SPEAKER;

  public void setHeadingSupplier(Supplier<Rotation2d> headingSupplier) {
    this.headingSupplier = Optional.of(headingSupplier);
  }

  public boolean isHeadingControlled() {
    return this.headingSupplier.isPresent();
  }

  public boolean isSpeakerControlled() {
    return this.headingSupplier.isPresent() && this.driveModeType == DriveModeType.SPEAKER;
  }

  public boolean isAmpControlled() {
    return this.headingSupplier.isPresent() && this.driveModeType == DriveModeType.AMP;
  }

  public DriveModeType getDriveModeType() {
    return this.driveModeType;
  }

  public void disableHeadingSupplier() {
    this.headingSupplier = Optional.empty();
  }

  public Rotation2d getHeadingAngle() {
    return headingSupplier.get().get();
  }

  public enum DriveModeType {
    AMP,
    SPEAKER,
  }

  public void setDriveMode(DriveModeType driveModeType) {
    this.driveModeType = driveModeType;
  }

  public void setAmpMode() {
    setHeadingSupplier(() -> Rotation2d.fromDegrees(90));
    setDriveMode(DriveController.DriveModeType.AMP);
  }

  public void setSpeakerMode(Supplier<Pose2d> poseSupplier) {
    setHeadingSupplier(
        () ->
            new Rotation2d(
                poseSupplier.get().getX()
                    - AllianceFlipUtil.apply(
                            FieldConstants.Speaker.centerSpeakerOpening.getTranslation())
                        .getX(),
                poseSupplier.get().getY()
                    - AllianceFlipUtil.apply(
                            FieldConstants.Speaker.centerSpeakerOpening.getTranslation())
                        .getY()));
    setDriveMode(DriveController.DriveModeType.SPEAKER);
  }

  public void disableDriveHeading() {
    disableHeadingSupplier();
  }
}
