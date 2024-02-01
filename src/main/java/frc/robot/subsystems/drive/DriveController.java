package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Optional;
import java.util.function.Supplier;

public class DriveController {
  private Optional<Supplier<Rotation2d>> headingSupplier = Optional.empty();
  private DriveModeType driveModeType = DriveModeType.STANDARD;

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
    STANDARD,
    AMP,
    SPEAKER,
    SHOOT_WHILE_MOVING,
    SPIN_TO_WIN,
    GAME_PICKUP,
  }

  public void setDriveMode(DriveModeType driveModeType) {
    this.driveModeType = driveModeType;
  }
}
