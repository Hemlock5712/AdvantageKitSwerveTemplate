// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

public class ColorSensorTester extends Command {
  /** Creates a new COlorSensorTester. */
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color noteColor = new Color(0.45, 0.4, 0.14);

  public ColorSensorTester() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_colorMatcher.addColorMatch(noteColor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /**
     * The method GetColor() returns a normalized color value from the sensor and can be useful if
     * outputting the color to an RGB LED or similar. To read the raw color, use GetRawColor().
     *
     * <p>The color sensor works best when within a few inches from an object in well lit conditions
     * (the built in LED is a big help here!). The farther an object is the more light from the
     * surroundings will bleed into the measurements and make it difficult to accurately determine
     * its color.
     */
    Color detectedColor = m_colorSensor.getColor();

    /** The sensor returns a raw IR value of the infrared light detected. */
    double IR = m_colorSensor.getIR();

    /** Open Smart Dashboard or Shuffleboard to see the color detected by the sensor. */
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR);

    /**
     * In addition to RGB IR values, the color sensor can also return an infrared proximity value.
     * The chip contains an IR led which will emit IR pulses and measure the intensity of the
     * return. When an object is close the value of the proximity will be large (max 2047 with
     * default settings) and will approach zero when the object is far away.
     *
     * <p>Proximity can be used to roughly approximate the distance of an object or provide a
     * threshold for when an object is close enough to provide accurate color values.
     */
    int proximity = m_colorSensor.getProximity();

    var match = m_colorMatcher.matchClosestColor(detectedColor);

    SmartDashboard.putNumber("Proximity", proximity);
    SmartDashboard.putNumber("confidence", match.confidence);

    SmartDashboard.putBoolean("sees note", match.confidence > 0.8);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
