package frc.robot.subsystems.ColorSensor;

import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ColorSensor extends SubsystemBase {

  private final ColorSensorIO colorSensorIO;
  private final ColorSensorIOInputsAutoLogged colorSensorInputs =
      new ColorSensorIOInputsAutoLogged();

  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color noteColor = new Color(0.45, 0.4, 0.14);

  public ColorSensor(ColorSensorIO colorSensorIO) {
    this.colorSensorIO = colorSensorIO;

    m_colorMatcher.addColorMatch(noteColor);
  }

  @Override
  public void periodic() {
  }

  public Color getColor() {
    return new Color(colorSensorInputs.red, colorSensorInputs.green, colorSensorInputs.blue);
  }

  public boolean detectNote() {
    colorSensorIO.updateInputs(colorSensorInputs);
    Logger.processInputs("ColorSensor", colorSensorInputs);

    final boolean output = colorSensorInputs.red > .33;
    Logger.recordOutput("DetectingNote", output);
    return output;
  }
}
