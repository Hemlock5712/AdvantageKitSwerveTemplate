package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;

public class NoteVisionSubsystem extends SubsystemBase {
    private static Pose3d LEFT_CAMERA_POS = new Pose3d(
            new Translation3d(0.3, -0.2, 0.45),
            new Rotation3d(0, Units.degreesToRadians(20), Units.degreesToRadians(-20))
    );
    private static Pose3d RIGHT_CAMERA_POS = new Pose3d(
            new Translation3d(0.3, 0.2, 0.45),
            new Rotation3d(0, Units.degreesToRadians(20), Units.degreesToRadians(-20))
    );

    private final NoteVisionIO leftNoteVisionIO;
    private final NoteVisionIO rightNoteVisionIO;
    private final NoteVisionIOInputsAutoLogged leftNoteVisionIOInputs = new NoteVisionIOInputsAutoLogged();
    private final NoteVisionIOInputsAutoLogged rightNoteVisionIOInputs = new NoteVisionIOInputsAutoLogged();

    public NoteVisionSubsystem(NoteVisionIO leftNoteVisionIO, NoteVisionIO rightNoteVisionIO) {
        this.leftNoteVisionIO = leftNoteVisionIO;
        this.rightNoteVisionIO = rightNoteVisionIO;
    }

    @Override
    public void periodic() {
        leftNoteVisionIO.updateInputs(leftNoteVisionIOInputs);
        rightNoteVisionIO.updateInputs(rightNoteVisionIOInputs);
    }

    private static List<Translation2d> calculateRelativeNotePoses(
            NoteVisionIO.NoteVisionIOInputs inputs,
            Pose3d cameraPose
    ) {
        final ArrayList<Translation2d> notePoses = new ArrayList<>();

        for (int i = 0; i < inputs.notePitches.length; i++) {
            double noteAngle = inputs.notePitches[i] + cameraPose.getRotation().getY();

            if (noteAngle >= 0) {
                continue;
            }

            double distanceFromCamera = cameraPose.getZ() / Math.tan(noteAngle);

            Translation2d noteToCameraPose = new Translation2d(distanceFromCamera, new Rotation2d(inputs.noteYaws[i]));

            notePoses.add(
                    noteToCameraPose
                        .rotateBy(cameraPose.getRotation().toRotation2d())
                        .plus(cameraPose.toPose2d().getTranslation())
            );
        }

        return notePoses;
    }
}
