// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.AllianceFlipUtil;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveToPointBuilder {
  private final Supplier<Pose2d> robotPoseSupplier;
  private final ArmSubsystem arm;
  private final ShooterSubsystem shooter;

  public DriveToPointBuilder(
      Supplier<Pose2d> robotPoseSupplier, ArmSubsystem arm, ShooterSubsystem shooter) {
    this.robotPoseSupplier = robotPoseSupplier;
    this.arm = arm;
    this.shooter = shooter;
  }

  public static Command driveTo(Pose2d targetPose) {
    return AutoBuilder.pathfindToPoseFlipped(
        targetPose, DriveConstants.pathPlannerConstraints, 0, 0.0);
  }

  public Command waitUntilNearPose(Pose2d targetPose, double distanceThreshold) {
    return Commands.waitUntil(
        () ->
            AllianceFlipUtil.apply(targetPose.getTranslation())
                    .getDistance(robotPoseSupplier.get().getTranslation())
                < distanceThreshold);
  }

  public Command raiseArmAndReadyShooterNearPose(
      Pose2d targetPose,
      double distanceThreshold,
      DoubleSupplier armPositionSupplier,
      DoubleSupplier shooterSpeedSupplier) {
    return waitUntilNearPose(targetPose, distanceThreshold)
        .andThen(
            Commands.parallel(ShooterCommands.runSpeed(shooter, shooterSpeedSupplier)),
            ArmCommands.autoArmToPosition(arm, armPositionSupplier));
  }
}
