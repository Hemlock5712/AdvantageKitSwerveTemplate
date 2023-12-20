package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.DriveUtils;
import java.util.List;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class DriveWithAutoDefenseSpin extends Command {

  private static final double DEADBAND = 0.1;
  private static final double DISTANCE_TO_SPIN_RATIO = -2;
  private static final double MAX_SPIN_PERCENT = .75;

  public DriveWithAutoDefenseSpin(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    super();
    this.drive = drive;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
  }

  Drive drive;
  DoubleSupplier xSupplier;
  DoubleSupplier ySupplier;
  DoubleSupplier omegaSupplier;
  DoubleSupplier defenseOmegaSupplier;
  DoubleSupplier fullOmegaSupplier;

  @Override
  public void initialize() {
    double linearMagnitude =
        MathUtil.applyDeadband(
            Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());

    Translation2d driveVector = new Translation2d(linearMagnitude * 10, linearDirection);

    drive.setDefenseDriveVector(driveVector);
    drive.setDefenseStartPose(drive.getPose());

    defenseOmegaSupplier =
        () ->
            DriveUtils.getDistanceFromVector(
                    drive.getDefenseDriveVector(),
                    DriveUtils.translatePointToNewOrigin(
                        drive.getPose().getTranslation(),
                        drive.getDefenseStartPose().getTranslation()))
                * DISTANCE_TO_SPIN_RATIO;
    fullOmegaSupplier =
        () ->
            Math.max(
                Math.min(defenseOmegaSupplier.getAsDouble(), MAX_SPIN_PERCENT), -MAX_SPIN_PERCENT);
  }

  @Override
  public void execute() {
    // Apply deadband
    double linearMagnitude =
        MathUtil.applyDeadband(
            Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
    double omega = MathUtil.applyDeadband(fullOmegaSupplier.getAsDouble(), DEADBAND);

    // Square values
    linearMagnitude = linearMagnitude * linearMagnitude;
    omega = Math.copySign(omega * omega, omega);

    // Calcaulate new linear velocity
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();

    // Convert to field relative speeds & send command
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
            omega * drive.getMaxAngularSpeedRadPerSec(),
            drive.getRotation()));
    // Below this is all just logging stuff
    Logger.recordOutput(
        "Defense/DistanceToVector", defenseOmegaSupplier.getAsDouble() / DISTANCE_TO_SPIN_RATIO);
    Trajectory driveVectorTrajectory =
        new Trajectory(
            List.of(
                new Trajectory.State(0, 0, 0, new Pose2d(), 0),
                new Trajectory.State(
                    0, 0, 0, new Pose2d(drive.getDefenseDriveVector(), new Rotation2d()), 0)));
    Trajectory translatedDriveVectorTrajectory =
        new Trajectory(
            List.of(
                new Trajectory.State(0, 0, 0, drive.getDefenseStartPose(), 0),
                new Trajectory.State(
                    0,
                    0,
                    0,
                    drive
                        .getDefenseStartPose()
                        .plus(new Transform2d(drive.getDefenseDriveVector(), new Rotation2d())),
                    0)));
    Logger.recordOutput("Defense/DriveVectorTranslated", translatedDriveVectorTrajectory);
    Logger.recordOutput(
        "Defense/TranslatedRobotPose",
        new Pose2d(
            DriveUtils.translatePointToNewOrigin(
                drive.getPose().getTranslation(), drive.getDefenseStartPose().getTranslation()),
            drive.getPose().getRotation()));
    Logger.recordOutput("Defense/DriveVector", driveVectorTrajectory);
  }

  @Override
  public void end(boolean interrupted) {
    drive.setDefenseDriveVector(null);
    drive.setDefenseStartPose(null);

    
    Logger.recordOutput("Defense/TranslatedRobotPose", new Pose2d());
    Logger.recordOutput("Defense/DriveVectorTranslated", new Trajectory());
    Logger.recordOutput("Defense/DriveVector", new Trajectory());
    Logger.recordOutput("Defense/DistanceToVector", 0.0);
  }
}
