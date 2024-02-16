package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;

public class ShootingBasedOnPoseCalculator {

  private ShootingBasedOnPoseCalculator() {}

  // This method assumes that the note's velocity is high enough that its trajectory can
  // be approximated by a straight line. It aims at the top of the speaker
  public static double calculateAngleInRadiansWithConstantVelocity(Pose2d robotPose) {
    Pose2d speakerPose = AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening);
    // Pythagorean theorem
    double distanceFromRobotToSpeaker =
        Math.sqrt(
            Math.pow((speakerPose.getX() - robotPose.getX()), 2)
                + Math.pow((speakerPose.getY() - robotPose.getY()), 2));

    return Math.atan2(FieldConstants.speakerTopHeight, distanceFromRobotToSpeaker);
  }
}
