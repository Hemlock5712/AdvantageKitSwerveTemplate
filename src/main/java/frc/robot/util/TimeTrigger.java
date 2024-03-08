package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TimeTrigger extends Trigger {
    public TimeTrigger(double secondsLeftInMatch) {
        super(() -> DriverStation.getMatchTime() < secondsLeftInMatch);
    }
}
