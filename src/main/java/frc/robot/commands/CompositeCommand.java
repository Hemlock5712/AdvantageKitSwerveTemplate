package frc.robot.commands;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants.Speaker;

public class CompositeCommand {
  Drive drive;
  Flywheel flywheel;
  InterpolatingDoubleTreeMap distanceMap = new InterpolatingDoubleTreeMap();

  public CompositeCommand(Drive drive, Flywheel flywheel) {
    this.drive = drive;
    this.flywheel = flywheel;

    // Populate the distance map with distance-speed pairs
    distanceMap.put(1.0, 10.0);
    distanceMap.put(2.3, 15.7);
    distanceMap.put(3.6, 21.9);
    distanceMap.put(4.9, 27.8);
    distanceMap.put(6.2, 33.6);
    distanceMap.put(7.5, 39.4);
  }

  public Command speakerShoot() {
    return Commands.run(
        () -> {
          double distance =
              drive
                  .getPose()
                  .getTranslation()
                  .getDistance(
                      AllianceFlipUtil.apply(Speaker.centerSpeakerOpening.getTranslation()));
          // Get the corresponding speed from the distance-speed map
          double speed = distanceMap.get(distance);

          // Run the flywheel at the calculated speed
          flywheel.runVelocity(speed);
        });
  }
}
