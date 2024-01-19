package frc.robot.util;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

public class ColorSensor {
    private final ColorSensorV3 m_colorSensor;
    private final ColorMatch m_colorMatcher = new ColorMatch();
    private final Color noteColor = new Color(0.45, 0.4, 0.14);

    ColorSensor() {

        this(I2C.Port.kOnboard);
    }

    ColorSensor(I2C.Port port) {
        m_colorSensor = new ColorSensorV3(port);
        m_colorMatcher.addColorMatch(noteColor);

    }

    public boolean detectNote(double confidence) {
        return m_colorMatcher.ma
    }
}
