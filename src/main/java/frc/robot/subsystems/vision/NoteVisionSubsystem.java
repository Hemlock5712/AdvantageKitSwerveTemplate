package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PoseLog;
import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class NoteVisionSubsystem extends SubsystemBase {
  private static final Pose3d CAMERA_POS =
      new Pose3d(
          new Translation3d(0.4, 0, 0.45), new Rotation3d(0, -Units.degreesToRadians(10), 0));
  private final NoteVisionIO noteVisionIO;
  private final NoteVisionIOInputsAutoLogged noteVisionIOInputs =
      new NoteVisionIOInputsAutoLogged();

  private Translation2d[] relativeNotePoses = {};

  private final PoseLog noVisionPoseLog;
  private final Supplier<Pose2d> currentRobotPoseNoVisionSupplier;

  public NoteVisionSubsystem(
      NoteVisionIO noteVisionIO,
      NoteVisionIO rightNoteVisionIO,
      PoseLog noVisionPoseLog,
      Supplier<Pose2d> currentRobotPoseNoVisionSupplier) {
    this.noteVisionIO = noteVisionIO;
    this.noVisionPoseLog = noVisionPoseLog;
    this.currentRobotPoseNoVisionSupplier = currentRobotPoseNoVisionSupplier;
  }

  @Override
  public void periodic() {
    noteVisionIO.updateInputs(noteVisionIOInputs);

    Logger.processInputs("NoteVision", noteVisionIOInputs);

    relativeNotePoses =
        calculateRelativeNotePoses(
            noteVisionIOInputs,
            CAMERA_POS,
            noVisionPoseLog.getPoseAtTime(noteVisionIOInputs.timeStampSeconds),
            currentRobotPoseNoVisionSupplier.get());

    Logger.recordOutput(
        "NoteVisionSubsystem/old robot pose",
        noVisionPoseLog.getPoseAtTime(noteVisionIOInputs.timeStampSeconds));
    Logger.recordOutput(
        "NoteVisionSubsystem/cur robot pose", currentRobotPoseNoVisionSupplier.get());

    Logger.recordOutput("NoteVisionSubsystem/note poses", relativeNotePoses);
  }

  private static Translation2d[] calculateRelativeNotePoses(
      NoteVisionIO.NoteVisionIOInputs inputs,
      Pose3d cameraPose,
      Pose2d oldRobotPose,
      Pose2d currentRobotPose) {
    final ArrayList<Translation2d> notePoses = new ArrayList<>();

    for (int i = 0; i < inputs.notePitches.length; i++) {
      double noteAngle = inputs.notePitches[i] + cameraPose.getRotation().getY();

      if (noteAngle >= 0) {
        continue;
      }

      double distanceFromCamera = cameraPose.getZ() / Math.tan(-noteAngle);

      Translation2d noteToCameraPose =
          new Translation2d(distanceFromCamera, new Rotation2d(inputs.noteYaws[i]));

      var noteRelativeToOldRobot =
          noteToCameraPose
              .rotateBy(cameraPose.getRotation().toRotation2d())
              .plus(cameraPose.toPose2d().getTranslation());
      var noteInOdometrySpace =
          projectRelativeNotePoseOntoRobotPose(noteRelativeToOldRobot, oldRobotPose);

      var noteRelativeToCurrentRobot =
          deprojectProctedNoteFromRobotPose(noteInOdometrySpace, currentRobotPose);

      notePoses.add(noteRelativeToCurrentRobot);
    }

    return notePoses.toArray(new Translation2d[0]);
  }

  @AutoLogOutput
  public Translation2d[] getNotesInRelativeSpace() {
    return relativeNotePoses;
  }

  public static Translation2d projectRelativeNotePoseOntoRobotPose(
      Translation2d noteInRelativeSpace, Pose2d robotPose) {
    return noteInRelativeSpace.rotateBy(robotPose.getRotation()).plus(robotPose.getTranslation());
  }

  public static Translation2d deprojectProctedNoteFromRobotPose(
      Translation2d noteInRobotSpace, Pose2d robotPose) {
    return new Pose2d(noteInRobotSpace, new Rotation2d()).relativeTo(robotPose).getTranslation();
  }

  public static Optional<Translation2d> getClosestNote(Translation2d[] notes) {
    Optional<Translation2d> closest = Optional.empty();

    for (Translation2d note : notes) {
      if (closest.isEmpty() || note.getNorm() < closest.get().getNorm()) {
        closest = Optional.of(note);
      }
    }

    return closest;
  }

  public Translation2d[] getNotesInGlobalSpace(Pose2d robotPose) {
    Translation2d[] notes = getNotesInRelativeSpace();
    for (int i = 0; i < notes.length; i++) {
      notes[i] = projectRelativeNotePoseOntoRobotPose(notes[i], robotPose);
    }

    return notes;
  }

  public Optional<Translation2d> getCurrentNote() {
    return NoteVisionSubsystem.getClosestNote(getNotesInRelativeSpace());
  }
}
