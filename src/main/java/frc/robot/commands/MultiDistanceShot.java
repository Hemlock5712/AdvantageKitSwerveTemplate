// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.util.AllianceFlipUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

/** A command that shoots game piece from multi-distance position from the target. */
public class MultiDistanceShot extends Command {
  Supplier<Pose2d> poseSupplier;
  Pose2d targetPose;
  Flywheel flywheel;
  InterpolatingDoubleTreeMap distanceMap = new InterpolatingDoubleTreeMap();

  double distance;
  double speed;

  /**
   * Creates a new MultiDistanceShot command.
   *
   * @param poseSupplier The supplier for the robot's current pose.
   * @param targetPose The target pose to shoot at.
   * @param flywheel The flywheel subsystem.
   */
  public MultiDistanceShot(Supplier<Pose2d> poseSupplier, Pose2d targetPose, Flywheel flywheel) {
    this.poseSupplier = poseSupplier;
    this.targetPose = targetPose;
    this.flywheel = flywheel;

    // Populate the distance map with distance-speed pairs
    distanceMap.put(1.0, 10.0);
    distanceMap.put(2.3, 15.7);
    distanceMap.put(3.6, 21.9);
    distanceMap.put(4.9, 27.8);
    distanceMap.put(6.2, 33.6);
    distanceMap.put(7.5, 39.4);
  }

  @Override
  public void initialize() {
    // Apply any necessary transformations to the target pose
    targetPose = AllianceFlipUtil.apply(targetPose);
  }

  @Override
  public void execute() {
    // Calculate the distance from the current pose to the target pose
    distance = poseSupplier.get().getTranslation().getDistance(targetPose.getTranslation());

    // Get the corresponding speed from the distance-speed map
    speed = distanceMap.get(distance);

    // Run the flywheel at the calculated speed
    flywheel.runVelocity(speed);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the flywheel when the command ends
    flywheel.stop();
  }

  @Override
  public boolean isFinished() {
    // The command never finishes on its own
    return false;
  }

  /**
   * Gets the distance from the current pose to the target pose.
   *
   * @return The distance in units.
   */
  @AutoLogOutput(key = "Shooter/DistanceToTarget")
  public double getDistance() {
    return distance;
  }

  /**
   * Gets the speed of the flywheel.
   *
   * @return The speed in units per second.
   */
  @AutoLogOutput(key = "Shooter/Speed")
  public double getSpeed() {
    return speed;
  }
}
