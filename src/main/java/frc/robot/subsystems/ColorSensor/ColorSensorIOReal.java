package frc.robot.subsystems.ColorSensor;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;

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
    inputs.red = colorSensor.getRed();
    inputs.blue = colorSensor.getBlue();
    inputs.green = colorSensor.getGreen();
    inputs.proximity = colorSensor.getProximity();
    inputs.IR = colorSensor.getIR();
  }
}
