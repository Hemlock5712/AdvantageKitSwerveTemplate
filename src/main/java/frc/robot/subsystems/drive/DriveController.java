package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import java.util.Optional;
import java.util.function.Supplier;

public class DriveController {
  private Optional<Supplier<Rotation2d>> headingSupplier = Optional.empty();
  private Supplier<Pose2d> poseSupplier = Pose2d::new;
  private DriveModeType driveModeType = DriveModeType.SPEAKER;

  // Sets the heading you want the robot to be facing
  public void setHeadingSupplier(Supplier<Rotation2d> headingSupplier) {
    this.headingSupplier = Optional.of(headingSupplier);
  }

  // Passes the current pose of the robot
  public void setPoseSupplier(Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;
  }

  // Checks if the heading is being controlled
  public boolean isHeadingControlled() {
    return this.headingSupplier.isPresent();
  }

  // Gets the current drive mode
  public Supplier<DriveModeType> getDriveModeType() {
    return () -> this.driveModeType;
  }

  // Gets the current heading angle
  public Supplier<Rotation2d> getHeadingAngle() {
    return headingSupplier.get();
  }

  // Sets the drive mode (amp or speaker mode)
  public void setDriveMode(DriveModeType driveModeType) {
    this.driveModeType = driveModeType;
    updateHeading();
  }

  // Toggles the drive mode
  public void toggleDriveMode() {
    if (getDriveModeType().get() == DriveModeType.AMP) {
      setDriveMode(DriveController.DriveModeType.SPEAKER);
    } else {
      setDriveMode(DriveController.DriveModeType.AMP);
    }
  }

  // Engages based on the current drive mode
  public void enableHeadingControl() {
    if (this.driveModeType == DriveModeType.AMP) {
      enableAmpHeading();
    } else {
      enableSpeakerHeading();
    }
  }

  // Disables the heading supplier (heading control is disabled)
  public void disableHeadingControl() {
    this.headingSupplier = Optional.empty();
  }

  // Updates the heading if it is being controlled
  private void updateHeading() {
    if (isHeadingControlled()) {
      enableHeadingControl();
    }
  }

  // Turns on heading control and sets to amp mode
  private void enableAmpHeading() {
    setHeadingSupplier(() -> Rotation2d.fromDegrees(90));
  }

  // Turns on heading control and sets to speaker mode
  private void enableSpeakerHeading() {
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
  }

  // Possible Drive Modes
  public enum DriveModeType {
    AMP,
    SPEAKER,
  }
}
