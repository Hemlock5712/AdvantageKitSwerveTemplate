// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.Drive;
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
    return driveTo(targetPose, 0);
  }

  public static Command driveTo(Pose2d targetPose, double endVelocity) {
    return AutoBuilder.pathfindToPoseFlipped(
        targetPose, DriveConstants.pathPlannerConstraints, endVelocity, 0.0);
  }

  public static Command driveToNoFlip(Pose2d targetPose) {
    return driveToNoFlip(targetPose, 0);
  }

  public static Command driveToNoFlip(Pose2d targetPose, double endVelocity) {
    return AutoBuilder.pathfindToPose(
        targetPose, DriveConstants.pathPlannerConstraints, endVelocity, 0.0);
  }

  public static Command align(
      Drive drive,
      Pose2d targetPose,
      double distanceTolerance,
      double angleToleranceRad,
      boolean flip) {
    final var flippedTargetPose = flip ? AllianceFlipUtil.apply(targetPose) : targetPose;
    return Commands.runEnd(
            () -> {
              final var pos = drive.getPose();

              final var translationOffset =
                  flippedTargetPose.getTranslation().minus(pos.getTranslation());
              final var rotationOffset = flippedTargetPose.getRotation().minus(drive.getRotation());

              var omega = drive.getThetaController().calculate(0, rotationOffset.getRadians());
              final var speedX =
                  DriveConstants.PPtranslationConstants.kP * translationOffset.getX();
              final var speedY =
                  DriveConstants.PPtranslationConstants.kP * translationOffset.getY();

              if (drive.getThetaController().atGoal()) {
                omega = 0;
              }

              final var chassisSpeeds =
                  ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, omega, pos.getRotation());

              drive.runVelocity(chassisSpeeds);
            },
            drive::stop,
            drive)
        .until(
            () -> {
              final var pos = drive.getPose();
              return pos.getTranslation().getDistance(flippedTargetPose.getTranslation())
                      < distanceTolerance
                  && Math.abs(pos.getRotation().minus(flippedTargetPose.getRotation()).getRadians())
                      < angleToleranceRad;
            });
  }

  public static Command driveToAndAlignNoFlip(
      Drive drive, Pose2d targetPose, double distanceTolerance, double angleTolerance) {
    return driveToNoFlip(targetPose)
        .andThen(align(drive, targetPose, distanceTolerance, angleTolerance, false));
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
