// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.LimelightTarget_Fiducial;

public class AprilTagVisionIOLimelight implements AprilTagVisionIO {

  String limelightName;

  public AprilTagVisionIOLimelight(String identifier) {
    limelightName = identifier;
    LimelightHelpers.setPipelineIndex(limelightName, 0);
  }

  public void updateInputs(AprilTagVisionIOInputs inputs) {
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      var estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
      inputs.estimatedPose = estimate.pose;
      inputs.captureTimestamp = estimate.timestampSeconds;
    } else {
      var estimate = LimelightHelpers.getBotPoseEstimate_wpiRed(limelightName);
      inputs.estimatedPose = estimate.pose;
      inputs.captureTimestamp = estimate.timestampSeconds;
    }
    inputs.valid = LimelightHelpers.getTV(limelightName);
    LimelightTarget_Fiducial[] tagID =
        LimelightHelpers.getLatestResults(limelightName).targetingResults.targets_Fiducials;
    int temp[] = new int[tagID.length];
    for (int i = 0; i < tagID.length; i++) {
      temp[i] = (int) tagID[i].fiducialID;
    }
    inputs.currentTags = temp;
  }
}
