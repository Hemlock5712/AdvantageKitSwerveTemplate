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
import org.littletonrobotics.junction.Logger;

public class AprilTagVisionIOLimelight implements AprilTagVisionIO {

  String limelightName;

  public AprilTagVisionIOLimelight(String identifier) {
    limelightName = identifier;
  }

  public void updateInputs(AprilTagVisionIOInputs inputs) {
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      inputs.estimatedPose = LimelightHelpers.getBotPose3d_wpiBlue(limelightName);
    } else {
      inputs.estimatedPose = LimelightHelpers.getBotPose3d_wpiRed(limelightName);
    }
    inputs.captureTimestamp =
        Logger.getRealTimestamp() - (LimelightHelpers.getLatency_Capture(limelightName) / 1000.0);
    inputs.valid = LimelightHelpers.getTV(limelightName);
    LimelightTarget_Fiducial[] tagID =
        LimelightHelpers.getLatestResults(limelightName).targetingResults.targets_Fiducials;
    for (int i = 0; i < tagID.length; i++) {
      inputs.currentTags.add(tagID[i].fiducialID);
    }
  }
}
