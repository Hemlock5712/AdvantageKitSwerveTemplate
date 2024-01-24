package frc.robot.util;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class CalculateSpeakerShooting {
  Drive drive;
  ShooterSubsystem shooter;

  public CalculateSpeakerShooting() {}
  ;

  // Uses the robot distance from speaker and shooter velocity to calculate angle of the arm
  public double calculateArmAngle() {
    return 0;
  }

  // Uses the robot distance from the speaker and arm angle to calculate shooter velocity
  public double calculateShooterVelocityRadPerSec() {
    return 0;
  }
}
