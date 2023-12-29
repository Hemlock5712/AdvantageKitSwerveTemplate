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
import lombok.Getter;

@Getter
public class AprilTagVisionIOLimelight implements AprilTagVisionIO {

  private final String limelightName;
  private final Alliance alliance;

  public AprilTagVisionIOLimelight(String identifier) {
    limelightName = identifier;
    LimelightHelpers.setPipelineIndex(limelightName, 0);
    alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
  }

  @Override
  public void updateInputs(AprilTagVisionIOInputs inputs) {
    LimelightHelpers.PoseEstimate estimate;
    if (alliance == Alliance.Blue) {
      estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
    } else {
      estimate = LimelightHelpers.getBotPoseEstimate_wpiRed(limelightName);
    }

    inputs.setEstimatedPose(estimate.pose);
    inputs.setCaptureTimestamp(estimate.timestampSeconds);
    inputs.setValid(LimelightHelpers.getTV(limelightName));

    LimelightTarget_Fiducial[] tagID =
        LimelightHelpers.getLatestResults(limelightName).targetingResults.targets_Fiducials;
    int[] temp = new int[tagID.length];
    for (int i = 0; i < tagID.length; i++) {
      temp[i] = (int) tagID[i].fiducialID;
    }
    inputs.setCurrentTags(temp);
  }
}
