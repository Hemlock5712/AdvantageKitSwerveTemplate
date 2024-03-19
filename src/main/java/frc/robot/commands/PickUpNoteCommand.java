package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class PickUpNoteCommand extends Command {
  private final Drive drive;
  private final Intake intake;
  private final Supplier<Optional<Translation2d>> relativeNoteSupplier;
  private final BooleanSupplier hasNote;

  public PickUpNoteCommand(
      Drive drive,
      Intake intake,
      Supplier<Optional<Translation2d>> relativeNoteSupplier,
      BooleanSupplier hasNote) {
    this.drive = drive;
    this.intake = intake;
    this.relativeNoteSupplier = relativeNoteSupplier;
    this.hasNote = hasNote;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.drive, this.intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    var currentNote = relativeNoteSupplier.get();

    if (currentNote.isEmpty()) {
      drive.runVelocity(ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, 1, new Rotation2d()));
      intake.stop();
      return;
    }

    var angle = currentNote.get().getAngle();
    double distanceToNote = currentNote.get().getNorm();

    var omega = drive.getThetaController().calculate(0, angle.getRadians());
    if (drive.getThetaController().atGoal()) {
      omega = 0;
    }
    omega =
        Math.copySign(
            Math.min(DriveConstants.NOTE_PICKUP_MAX_TURN_SPEED.get(), Math.abs(omega)), omega);

    double speed =
        MathUtil.clamp(
            distanceToNote * DriveConstants.NOTE_PICKUP_DISTANCE_TO_SPEED_MULT.get(),
            DriveConstants.NOTE_PICKUP_MIN_SPEED.get(),
            DriveConstants.NOTE_PICKUP_MAX_SPEED.get());

    if (distanceToNote < 1 && !drive.getThetaController().atGoal()) {
      speed = 0;
    }

    double speedx = speed * angle.getCos();
    double speedy = speed * angle.getSin();

    var speeds = ChassisSpeeds.fromRobotRelativeSpeeds(speedx, speedy, omega, new Rotation2d());

    drive.runVelocity(speeds);

    if (distanceToNote < 2) {
      intake.setVoltage(IntakeConstants.INTAKE_VOLTAGE.get());
    } else {
      intake.stop();
    }
  }

  @Override
  public boolean isFinished() {
    return hasNote.getAsBoolean();
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
    drive.stop();
  }
}
