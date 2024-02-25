// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** A command that runs pathfindThenFollowPath based on the current drive mode. */
public class PathFinderAndFollow extends Command {
  private Command scoreCommand;
  private Command pathRun;
  private PathPlannerPath targetPath;

  /**
   * Creates a new PathFinderAndFollow command.
   */
  public PathFinderAndFollow(PathPlannerPath targetPath) {
    this.targetPath = targetPath;
  }

  @Override
  public void initialize() {
    runNewAutoPath();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    scoreCommand.cancel();
  }

  @Override
  public boolean isFinished() {
    return pathRun.isFinished();
  }

  /** Runs a new autonomous path based on the current drive mode. */
  public void runNewAutoPath() {
    PathConstraints constraints =
        new PathConstraints(2.0, 2.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
    pathRun = AutoBuilder.pathfindThenFollowPath(targetPath, constraints, 0.0);
    scoreCommand = Commands.sequence(pathRun);
    scoreCommand.schedule();
  }
}
