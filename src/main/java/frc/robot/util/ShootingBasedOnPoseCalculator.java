package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ShootingBasedOnPoseCalculator {
  private final Pose2d speakerPose = new Pose2d(); // todo find the pose of the speaker

  private ShootingBasedOnPoseCalculator() {}

  public static double calculateAngleInRadiansWithConstantVelocity(
      ArmSubsystem arm, Pose2d robotPose) {
    return 0;
  }
}
