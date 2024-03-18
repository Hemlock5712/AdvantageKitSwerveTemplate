package frc.robot.subsystems.beamBreak;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.IntConsumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class BeamBreakIOSim implements BeamBreakIO {
  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<Translation2d[]> noteLocationSupplier;
  private final DoubleSupplier intakeSpeedSupplier;
  private final DoubleSupplier shooterSpeedSupplier;
  private final IntConsumer remoteNote;

  public BeamBreakIOSim(
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<Translation2d[]> noteLocationSupplier,
      DoubleSupplier intakeSpeedSupplier,
      DoubleSupplier shooterSpeedSupplier,
      IntConsumer remoteNote) {
    this.robotPoseSupplier = robotPoseSupplier;
    this.noteLocationSupplier = noteLocationSupplier;
    this.intakeSpeedSupplier = intakeSpeedSupplier;
    this.shooterSpeedSupplier = shooterSpeedSupplier;
    this.remoteNote = remoteNote;
  }

  private static Optional<Integer> findClosestNoteIndex(
      Pose2d robotPose, Translation2d[] noteTranslations) {
    if (noteTranslations.length == 0) {
      return Optional.empty();
    }
    int closestIndex = 0;
    double closestDistance = robotPose.getTranslation().getDistance(noteTranslations[0]);
    for (int i = 1; i < noteTranslations.length; i++) {
      double currentDistance = robotPose.getTranslation().getDistance(noteTranslations[i]);
      if (currentDistance < closestDistance) {
        closestIndex = i;
        closestDistance = currentDistance;
      }
    }

    return Optional.of(closestIndex);
  }

  @Override
  public void updateInputs(BeamBreakIOInputs inputs) {
    final String logPrefix = "beambreaksim/";
    final boolean isIntaking = intakeSpeedSupplier.getAsDouble() > 0;
    Logger.recordOutput(logPrefix + "intaking", isIntaking);
    if (inputs.triggered) {
      final boolean isExtaking = intakeSpeedSupplier.getAsDouble() < 0;
      Logger.recordOutput(logPrefix + "extaking", isExtaking);
      final boolean isShooterSpinning = shooterSpeedSupplier.getAsDouble() > 0;
      Logger.recordOutput(logPrefix + "shooting", isShooterSpinning);
      if (isExtaking || (isIntaking && isShooterSpinning)) {
        inputs.triggered = false;
      }
    } else {
      final Pose2d robotPose = robotPoseSupplier.get();
      final Translation2d[] notes = noteLocationSupplier.get();
      final Optional<Integer> closestNoteIndex = findClosestNoteIndex(robotPose, notes);

      Logger.recordOutput(logPrefix + "robotpose", robotPose);
      Logger.recordOutput(logPrefix + "notes", notes);
      Logger.recordOutput(logPrefix + "has closest note", closestNoteIndex.isPresent());

      if (closestNoteIndex.isEmpty()) {
        return;
      }
      Logger.recordOutput(logPrefix + "closest note i", closestNoteIndex.get());
      Logger.recordOutput(logPrefix + "closest note", notes[closestNoteIndex.get()]);
      final boolean inRangeOfNote =
          robotPose.getTranslation().getDistance(notes[closestNoteIndex.get()]) < 0.5;
      Logger.recordOutput(logPrefix + "in range of note", inRangeOfNote);
      Logger.recordOutput(
          logPrefix + "note distANCE",
          robotPose.getTranslation().getDistance(notes[closestNoteIndex.get()]));
      if (inRangeOfNote && isIntaking) {
        inputs.triggered = true;
        remoteNote.accept(closestNoteIndex.get());
      }
    }
  }
}
