package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.stream.Stream;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class NoteVisionSubsystem extends SubsystemBase {
  private static final Pose3d LEFT_CAMERA_POS =
      new Pose3d(
          new Translation3d(0.3, 0.2, 0.45),
          new Rotation3d(0, -Units.degreesToRadians(30), Units.degreesToRadians(40)));
  private static final Pose3d RIGHT_CAMERA_POS =
      new Pose3d(
          new Translation3d(0.3, -0.2, 0.45),
          new Rotation3d(0, -Units.degreesToRadians(30), -Units.degreesToRadians(40)));

  private final NoteVisionIO leftNoteVisionIO;
  private final NoteVisionIO rightNoteVisionIO;
  private final NoteVisionIOInputsAutoLogged leftNoteVisionIOInputs =
      new NoteVisionIOInputsAutoLogged();
  private final NoteVisionIOInputsAutoLogged rightNoteVisionIOInputs =
      new NoteVisionIOInputsAutoLogged();

  public NoteVisionSubsystem(NoteVisionIO leftNoteVisionIO, NoteVisionIO rightNoteVisionIO) {
    this.leftNoteVisionIO = leftNoteVisionIO;
    this.rightNoteVisionIO = rightNoteVisionIO;
  }

  @Override
  public void periodic() {
    leftNoteVisionIO.updateInputs(leftNoteVisionIOInputs);
    rightNoteVisionIO.updateInputs(rightNoteVisionIOInputs);

    Logger.processInputs("NoteVisionSubsystem/lefty", leftNoteVisionIOInputs);
    Logger.processInputs("NoteVisionSubsystem/righty", rightNoteVisionIOInputs);

    Logger.recordOutput(
        "NoteVisionSubsystem/lefty",
        calculateRelativeNotePoses(leftNoteVisionIOInputs, LEFT_CAMERA_POS));
    Logger.recordOutput(
        "NoteVisionSubsystem/righty",
        calculateRelativeNotePoses(rightNoteVisionIOInputs, RIGHT_CAMERA_POS));
  }

  private static Translation2d[] calculateRelativeNotePoses(
      NoteVisionIO.NoteVisionIOInputs inputs, Pose3d cameraPose) {
    final ArrayList<Translation2d> notePoses = new ArrayList<>();

    for (int i = 0; i < inputs.notePitches.length; i++) {
      double noteAngle = inputs.notePitches[i] + cameraPose.getRotation().getY();

      if (noteAngle >= 0) {
        continue;
      }

      double distanceFromCamera = cameraPose.getZ() / Math.tan(-noteAngle);

      Translation2d noteToCameraPose =
          new Translation2d(distanceFromCamera, new Rotation2d(inputs.noteYaws[i]));

      notePoses.add(
          noteToCameraPose
              .rotateBy(cameraPose.getRotation().toRotation2d())
              .plus(cameraPose.toPose2d().getTranslation()));
    }

    return notePoses.toArray(new Translation2d[0]);
  }

  @AutoLogOutput
  public Translation2d[] getNotes() {
    Translation2d[] leftNotes = calculateRelativeNotePoses(leftNoteVisionIOInputs, LEFT_CAMERA_POS);
    Translation2d[] rightNotes =
        calculateRelativeNotePoses(rightNoteVisionIOInputs, RIGHT_CAMERA_POS);

    return Stream.concat(Arrays.stream(leftNotes), Arrays.stream(rightNotes))
        .toArray(Translation2d[]::new);
  }

  public static Translation2d getNoteInGlobalSpace(Translation2d note, Pose2d robotPose) {
    return note.rotateBy(robotPose.getRotation()).plus(robotPose.getTranslation());
  }

  public static Translation2d getClosestNote(Translation2d[] notes) {
    Translation2d closest = null;

    for (Translation2d note : notes) {
      if (closest == null || note.getNorm() < closest.getNorm()) {
        closest = note;
      }
    }

    return closest;
  }

  public Translation2d[] getNotesInGlobalSpace(Pose2d robotPose) {
    Translation2d[] notes = getNotes();
    for (int i = 0; i < notes.length; i++) {
      notes[i] = getNoteInGlobalSpace(notes[i], robotPose);
    }

    return notes;
  }
}
