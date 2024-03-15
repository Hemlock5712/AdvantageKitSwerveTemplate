package frc.robot.subsystems.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PoseLog;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class NoteVisionSubsystem extends SubsystemBase {
  private final NoteVisionIO noteVisionIO;
  private final NoteVisionIOInputsAutoLogged noteVisionIOInputs =
      new NoteVisionIOInputsAutoLogged();

  private TimestampedNote[] notesInOdometrySpace = {};

  private final PoseLog noVisionPoseLog;
  private final Supplier<Pose2d> currentRobotPoseNoVisionSupplier;
  private double lastTimestamp = 0;

  public record TimestampedNote(Translation2d pose, double timestamp) {}

  public NoteVisionSubsystem(
      NoteVisionIO noteVisionIO,
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

    if (lastTimestamp == noteVisionIOInputs.timeStampSeconds) {
      return;
    } else {
      lastTimestamp = noteVisionIOInputs.timeStampSeconds;
    }

    var oldNotes = notesInOdometrySpace;
    var newNotes =
        calculateNotesInOdometrySpace(
            noteVisionIOInputs,
            NoteVisionConstants.CAMERA_POS,
            noVisionPoseLog.getPoseAtTime(noteVisionIOInputs.timeStampSeconds));

    ArrayList<TimestampedNote> oldNotesInCamera = new ArrayList<>();
    ArrayList<TimestampedNote> oldNotesOutOfCamera = new ArrayList<>();

    splitOldNotesInCameraView(
        noVisionPoseLog.getPoseAtTime(noteVisionIOInputs.timeStampSeconds),
        NoteVisionConstants.CAMERA_POS.toPose2d(),
        oldNotes,
        oldNotesInCamera,
        oldNotesOutOfCamera);

    var currentNotes =
        updateNotes(oldNotesInCamera, groupNotes(newNotes), noteVisionIOInputs.timeStampSeconds);

    filterOldUnseenNotes(oldNotesOutOfCamera, noteVisionIOInputs.timeStampSeconds);

    currentNotes.addAll(oldNotesOutOfCamera);

    groupNoteRecords(currentNotes);

    notesInOdometrySpace = currentNotes.toArray(TimestampedNote[]::new);

    Logger.recordOutput(
        "NoteVisionSubsystem/old robot pose",
        noVisionPoseLog.getPoseAtTime(noteVisionIOInputs.timeStampSeconds));
    Logger.recordOutput(
        "NoteVisionSubsystem/cur robot pose", currentRobotPoseNoVisionSupplier.get());

    Logger.recordOutput("NoteVisionSubsystem/note poses", getNotesInRelativeSpace());
    Logger.recordOutput(
        "NoteVisionSubsystem/note poses odometry in camera",
        oldNotesInCamera.stream().map(note -> note.pose).toArray(Translation2d[]::new));
    Logger.recordOutput(
        "NoteVisionSubsystem/note poses odometry out of camera",
        oldNotesOutOfCamera.stream().map(note -> note.pose).toArray(Translation2d[]::new));
    Logger.recordOutput(
        "NoteVisionSubsystem/note poses odometry space", newNotes.toArray(new Translation2d[0]));
  }

  private static void filterOldUnseenNotes(
      ArrayList<TimestampedNote> oldNotesOutOfCamera, double currentTime) {
    for (int i = 0; i < oldNotesOutOfCamera.size(); ) {
      if (oldNotesOutOfCamera.get(i).timestamp
          < currentTime - NoteVisionConstants.OUT_OF_CAMERA_EXPIRATION) {
        oldNotesOutOfCamera.remove(i);
      } else {
        i++;
      }
    }
  }

  private static void splitOldNotesInCameraView(
      Pose2d robotPose,
      Pose2d cameraPose2d,
      TimestampedNote[] oldNotes,
      List<TimestampedNote> inCameraRange,
      List<TimestampedNote> outOfCameraRange) {
    for (var note : oldNotes) {
      if (canCameraSeeNote(
          cameraPose2d, deprojectProjectedNoteFromRobotPose(note.pose, robotPose))) {
        inCameraRange.add(note);
      } else {
        outOfCameraRange.add(note);
      }
    }
  }

  private static boolean canCameraSeeNote(Pose2d cameraPose, Translation2d note) {
    double distance = note.getDistance(cameraPose.getTranslation());
    if (distance > NoteVisionConstants.MAX_CAMERA_DISTANCE
        || distance < NoteVisionConstants.MIN_CAMERA_DISTANCE) {
      return false;
    }
    Rotation2d angle = deprojectProjectedNoteFromRobotPose(note, cameraPose).getAngle();
    return MathUtil.isNear(0, angle.getDegrees(), NoteVisionConstants.LIFECAM_3000_HFOV / 2);
  }

  private static ArrayList<TimestampedNote> updateNotes(
      List<TimestampedNote> oldNotesInCamera,
      List<Translation2d> newNotes,
      double timeStampSeconds) {

    for (var oldNote : oldNotesInCamera) {
      for (int i = 0; i < newNotes.size(); i++) {
        if (areNotesSame(oldNote.pose, newNotes.get(i))) {
          newNotes.set(i, averageNotes(oldNote.pose, newNotes.get(i)));
          break;
        }
      }
    }

    var currentNotes =
        new ArrayList<>(
            newNotes.stream().map(note -> new TimestampedNote(note, timeStampSeconds)).toList());

    var recentNotes =
        oldNotesInCamera.stream()
            .filter(
                note ->
                    note.timestamp < timeStampSeconds - NoteVisionConstants.IN_CAMERA_EXPIRATION)
            .toList();

    currentNotes.addAll(recentNotes);

    return currentNotes;
  }

  private static boolean areNotesSame(Translation2d a, Translation2d b) {
    return a.getDistance(b) < NoteVisionConstants.NOTE_GROUPING_TOLERANCE;
  }

  private static Translation2d averageNotes(Translation2d a, Translation2d b) {
    return a.plus(b).div(2);
  }

  private static TimestampedNote averageNotes(TimestampedNote a, TimestampedNote b) {
    return new TimestampedNote(a.pose.plus(b.pose).div(2), Math.max(a.timestamp, b.timestamp));
  }

  private static void groupNoteRecords(List<TimestampedNote> notes) {
    outer:
    for (int i = 0; i < notes.size() - 1; ) {
      for (int j = i + 1; j < notes.size(); j++) {
        if (areNotesSame(notes.get(i).pose, notes.get(j).pose)) {
          notes.set(i, averageNotes(notes.get(i), notes.get(j)));
          notes.remove(j);
          continue outer;
        }
      }

      i++;
    }
  }

  private static List<Translation2d> groupNotes(List<Translation2d> noteTranslations) {
    ArrayList<Translation2d> notes = new ArrayList<>(noteTranslations);

    outer:
    for (int i = 0; i < notes.size() - 1; ) {
      for (int j = i + 1; j < notes.size(); j++) {
        if (areNotesSame(notes.get(i), notes.get(j))) {
          notes.set(i, averageNotes(notes.get(i), notes.get(j)));
          notes.remove(j);
          continue outer;
        }
      }

      i++;
    }

    return notes;
  }

  private static List<Translation2d> calculateNotesInOdometrySpace(
      NoteVisionIO.NoteVisionIOInputs inputs, Pose3d cameraPose, Pose2d oldRobotPose) {
    final ArrayList<Translation2d> notes = new ArrayList<>();

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

      notes.add(noteInOdometrySpace);
    }

    return notes;
  }

  @AutoLogOutput
  public Translation2d[] getNotesInRelativeSpace() {
    return Arrays.stream(notesInOdometrySpace)
        .map(
            odometrySpaceNote ->
                deprojectProjectedNoteFromRobotPose(
                    odometrySpaceNote.pose, currentRobotPoseNoVisionSupplier.get()))
        .toArray(Translation2d[]::new);
  }

  public static Translation2d projectRelativeNotePoseOntoRobotPose(
      Translation2d noteInRelativeSpace, Pose2d robotPose) {
    return noteInRelativeSpace.rotateBy(robotPose.getRotation()).plus(robotPose.getTranslation());
  }

  public static Translation2d deprojectProjectedNoteFromRobotPose(
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
