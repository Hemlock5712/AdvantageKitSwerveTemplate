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
import frc.robot.subsystems.drive.DriveController.DriveModeType;
import java.util.function.Supplier;

/** A command that runs pathfindThenFollowPath based on the current drive mode. */
public class PathFinderAndFollow extends Command {
  private final Supplier<DriveModeType> driveModeSupplier;
  private Command scoreCommand;
  private Command pathRun;
  private DriveModeType driveMode;

  /**
   * Creates a new PathFinderAndFollow command.
   *
   * @param driveModeSupplier a supplier for the drive mode type
   */
  public PathFinderAndFollow(Supplier<DriveModeType> driveModeSupplier) {
    this.driveModeSupplier = driveModeSupplier;
  }

  @Override
  public void initialize() {
    runNewAutonPath();
  }

  @Override
  public void execute() {
    DriveModeType currentDriveMode = driveModeSupplier.get();
    if (driveMode != currentDriveMode) {
      scoreCommand.cancel();
      runNewAutonPath();
    }
  }

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
  public void runNewAutonPath() {
    driveMode = driveModeSupplier.get();
    String pathName =
        driveMode == DriveModeType.SPEAKER ? "Speaker Placement Path" : "Amp Placement Path";
    PathPlannerPath ampPath = PathPlannerPath.fromPathFile(pathName);
    PathConstraints constraints =
        new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
    pathRun = AutoBuilder.pathfindThenFollowPath(ampPath, constraints, 0.0);
    scoreCommand = Commands.sequence(pathRun);
    scoreCommand.schedule();
  }
}
