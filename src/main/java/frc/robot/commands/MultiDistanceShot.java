// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.interpolation.InterpolationMaps;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** A command that shoots game piece from multi-distance position from the target. */
public class MultiDistanceShot extends Command {
  private final Supplier<Pose2d> poseSupplier;
  private final Pose2d targetPose;
  private final ShooterSubsystem shooter;
  private final ArmSubsystem arm;

  /**
   * Creates a new MultiDistanceShot command.
   *
   * @param poseSupplier The supplier for the robot's current pose.
   * @param targetPose The target pose to shoot at.
   * @param shooter shooter subsystem
   */
  public MultiDistanceShot(
      Supplier<Pose2d> poseSupplier,
      Pose2d targetPose,
      ShooterSubsystem shooter,
      ArmSubsystem arm) {
    this.poseSupplier = poseSupplier;
    this.targetPose = AllianceFlipUtil.apply(targetPose);
    this.shooter = shooter;
    this.arm = arm;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Calculate the distance from the current pose to the target pose
    double distance = poseSupplier.get().getTranslation().getDistance(targetPose.getTranslation());

    // Get the corresponding speed from the distance-speed map
    double speed = InterpolationMaps.shooterDistanceToVelocity.get(distance);
    double armAngle = InterpolationMaps.getShooterDistanceToArmAngle.get(distance);

    Logger.recordOutput("MultiDistanceShot/speed", speed);
    Logger.recordOutput("MultiDistanceShot/arm angle", armAngle);

    // Run the flywheel at the calculated speed
    shooter.runVelocity(speed);
    arm.setPositionRad(armAngle);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the flywheel when the command ends
    arm.stop();
    shooter.stop();
  }

  @Override
  public boolean isFinished() {
    // The command never finishes on its own
    return false;
  }
}
