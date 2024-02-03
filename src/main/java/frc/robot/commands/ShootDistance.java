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

public class ShootDistance extends Command {
  Supplier<Pose2d> poseSupplier;
  Pose2d targetPose;
  Flywheel flywheel;
  InterpolatingDoubleTreeMap distanceMap = new InterpolatingDoubleTreeMap();

  // Depending on how you are going to use this command, you may want to set a deafult
  // distance/speed in FindDistance and end
  double distance;
  double speed;

  /** Creates a new FindDistance. */
  public ShootDistance(Supplier<Pose2d> poseSupplier, Pose2d targetPose, Flywheel flywheel) {
    this.poseSupplier = poseSupplier;
    this.targetPose = targetPose;
    this.flywheel = flywheel;
    // First value would be distance from target, second value would be speed or angle depending on
    // application
    // In this case RPM
    distanceMap.put(1.0, 10.0);
    distanceMap.put(2.3, 15.7);
    distanceMap.put(3.6, 21.9);
    distanceMap.put(4.9, 27.8);
    distanceMap.put(6.2, 33.6);
    distanceMap.put(7.5, 39.4);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetPose = AllianceFlipUtil.apply(targetPose);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    distance = poseSupplier.get().getTranslation().getDistance(targetPose.getTranslation());
    speed = distanceMap.get(distance);
    flywheel.runVelocity(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheel.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @AutoLogOutput(key = "Shooter/DistanceToTarget")
  public double getDistance() {
    return distance;
  }

  @AutoLogOutput(key = "Shooter/Speed")
  public double getSpeed() {
    return speed;
  }
}
