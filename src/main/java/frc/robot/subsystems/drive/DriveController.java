package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;

/**
 * The DriveController class represents a controller for the robot's drive system. It provides
 * methods to control the heading and drive mode of the robot.
 */
public class DriveController {
  public static DriveController instance;

  private Rotation2d headingSupplier = Rotation2d.fromDegrees(90);
  private Boolean headingControl = false;
  private DriveModeType driveModeType = DriveModeType.AMP;

  public static DriveController getInstance() {
    if (instance == null) {
      instance = new DriveController();
    }
    return instance;
  }

  /**
   * Sets the heading supplier that provides the desired heading for the robot.
   *
   * @param headingSupplier The supplier that provides the desired heading.
   */
  public void setHeading(Rotation2d headingSupplier) {
    this.headingSupplier = headingSupplier;
  }

  /**
   * Checks if the heading is being controlled.
   *
   * @return True if the heading is being controlled, false otherwise.
   */
  public boolean isHeadingControlled() {
    return this.headingControl;
  }

  /**
   * Gets the current drive mode.
   *
   * @return The supplier that provides the current drive mode.
   */
  public DriveModeType getDriveModeType() {
    return this.driveModeType;
  }

  /**
   * Gets the current heading angle.
   *
   * @return The supplier that provides the current heading angle.
   */
  public Rotation2d getHeadingAngle() {
    return this.headingSupplier;
  }

  /**
   * Sets the drive mode.
   *
   * @param driveModeType The drive mode to set.
   */
  public void setDriveMode(DriveModeType driveModeType) {
    this.driveModeType = driveModeType;
  }

  /** Toggles the drive mode between AMP and SPEAKER. */
  public void toggleDriveMode() {
    if (getDriveModeType() == DriveModeType.AMP) {
      setDriveMode(DriveController.DriveModeType.SPEAKER);
    } else {
      setDriveMode(DriveController.DriveModeType.AMP);
    }
  }

  /** Enables heading control based on the current drive mode. */
  public void enableHeadingControl() {
    this.headingControl = true;
  }

  /** Disables heading control (heading control is disabled). */
  public void disableHeadingControl() {
    this.headingControl = false;
  }

  /**
   * Calculates the heading based on the current drive mode.
   *
   * @param pose The current pose of the robot.
   */
  public void calculateHeading(Pose2d pose) {
    switch (getDriveModeType()) {
      case AMP:
        calculateAmpHeading();
        break;
      case SPEAKER:
        calculateSpeakerHeading(pose);
        break;
    }
  }

  /** Turns on heading control and sets the heading to AMP mode (90 degrees). */
  private void calculateAmpHeading() {
    setHeading(Rotation2d.fromDegrees(90));
  }

  /** Turns on heading control and sets the heading to SPEAKER mode. */
  private void calculateSpeakerHeading(Pose2d pose) {
    setHeading(
        new Rotation2d(
            pose.getX()
                - AllianceFlipUtil.apply(
                        FieldConstants.Speaker.centerSpeakerOpening.getTranslation())
                    .getX(),
            pose.getY()
                - AllianceFlipUtil.apply(
                        FieldConstants.Speaker.centerSpeakerOpening.getTranslation())
                    .getY()));
  }

  /** Possible Drive Modes. */
  public enum DriveModeType {
    AMP,
    SPEAKER,
  }
}
