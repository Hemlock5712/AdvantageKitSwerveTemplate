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

public class AutoRun extends Command {
  private Supplier<DriveModeType> driveModeSupplier;
  private Command scoreCommand;
  private Command pathRun;
  private DriveModeType driveMode;

  /** Creates a new AutoRun. */
  public AutoRun(Supplier<DriveModeType> driveModeSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveModeSupplier = driveModeSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveMode = driveModeSupplier.get();
    String pathName = "Amp Placement Path";
    if (driveMode == DriveModeType.SPEAKER) {
      pathName = "Speaker Placement Path";
    }
    PathPlannerPath ampPath = PathPlannerPath.fromPathFile(pathName);
    PathConstraints constraints =
        new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
    pathRun = AutoBuilder.pathfindThenFollowPath(ampPath, constraints, 0.0);
    scoreCommand = Commands.sequence(pathRun);
    scoreCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (driveMode != driveModeSupplier.get()) {
      scoreCommand.cancel();
      driveMode = driveModeSupplier.get();
      initialize();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    scoreCommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathRun.isFinished();
  }
}
