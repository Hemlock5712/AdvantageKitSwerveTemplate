// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.AllianceFlipUtil;

public class DriveToPoint extends Command {
  protected final Pose2d targetPose;
  protected Command pathRun;

  public DriveToPoint(Pose2d targetPose) {
    this.targetPose = targetPose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pathRun =
        AutoBuilder.pathfindToPose(
            AllianceFlipUtil.apply(targetPose), DriveConstants.pathPlannerConstraints, 0, 0.0);
    pathRun.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    pathRun.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // This should be the last command in the sequence
    return pathRun.isFinished();
  }
}
