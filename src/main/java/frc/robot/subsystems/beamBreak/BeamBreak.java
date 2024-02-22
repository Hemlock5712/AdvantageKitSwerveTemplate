package frc.robot.subsystems.beamBreak;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class BeamBreak extends SubsystemBase {

  private final BeamBreakIO beamBreakIO;
  private final BeamBreakIOInputsAutoLogged beamBreakInputs = new BeamBreakIOInputsAutoLogged();

  public BeamBreak(BeamBreakIO beamBreakIO) {
    this.beamBreakIO = beamBreakIO;
  }

  @Override
  public void periodic() {
    beamBreakIO.updateInputs(beamBreakInputs);
    Logger.processInputs("BeamBreak", beamBreakInputs);
  }

  public boolean detectNote() {
    return beamBreakInputs.triggered;
  }
}
