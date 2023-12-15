package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.DriveUtils;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class DriveWithAutoDefenseSpin extends Command {

  private static final double DEADBAND = 0.1;

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
    defenseOmegaSupplier =
        () ->
            DriveUtils.getDistanceFromVector(
                    drive.getDefenseDriveVector(), drive.getPose().getTranslation())
                * Constants.DEFENSE_DISTANCE_TO_SPIN_RATIO;
    fullOmegaSupplier = () -> defenseOmegaSupplier.getAsDouble() * omegaSupplier.getAsDouble();
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
    Logger.recordOutput("Defense/Omega", defenseOmegaSupplier.getAsDouble());
    Logger.recordOutput("Defense/Vector", drive.getDefenseDriveVector());
  }

  @Override
  public void end(boolean interrupted) {
    drive.setDefenseDriveVector(null);
  }
}
