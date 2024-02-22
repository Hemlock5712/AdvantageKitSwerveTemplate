package frc.robot.subsystems.beamBreak;

import edu.wpi.first.wpilibj.DigitalInput;

public class BeamBreakIOReal implements BeamBreakIO {
  private final DigitalInput beamBreakSensor;

  public BeamBreakIOReal() {
    beamBreakSensor = new DigitalInput(BeamBreakConstants.BEAM_BREAK_SENSOR_PORT);
  }

  @Override
  public void updateInputs(BeamBreakIOInputs inputs) {
    inputs.triggered = !beamBreakSensor.get();
  }
}
