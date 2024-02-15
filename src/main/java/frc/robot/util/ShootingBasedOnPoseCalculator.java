package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ShootingBasedOnPoseCalculator {

  private ShootingBasedOnPoseCalculator() {}

  public static double calculateAngleInRadiansWithConstantVelocity(Pose2d robotPose) {
    Pose2d speakerPose = AllianceFlipUtil.apply(
            FieldConstants.Speaker.centerSpeakerOpening);
    //Pythagorean theorem
    double distanceFromRobotToSpeaker =
            Math.sqrt(Math.pow((speakerPose.getX() - robotPose.getX()), 2) +
                    Math.pow((speakerPose.getY() - robotPose.getY()), 2));

    return Math.atan2(FieldConstants.speakerTopHeight, distanceFromRobotToSpeaker);
  }
}
