package frc.robot.subsystems.ColorSensor;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import org.littletonrobotics.junction.Logger;

public class ColorSensorIOReal implements ColorSensorIO {
  private final ColorSensorV3 colorSensor;

  public ColorSensorIOReal() {
    this(I2C.Port.kOnboard);
  }

  ColorSensorIOReal(I2C.Port port) {
    colorSensor = new ColorSensorV3(port);
  }

  @Override
  public void updateInputs(ColorSensorIOInputs inputs) {
    Logger.recordOutput("colorsensorconnected", colorSensor.isConnected());
    Color color = colorSensor.getColor();
    inputs.red = color.red;
    inputs.blue = color.blue;
    inputs.green = color.green;
    inputs.proximity = colorSensor.getProximity();
    inputs.IR = colorSensor.getIR();
  }
}
