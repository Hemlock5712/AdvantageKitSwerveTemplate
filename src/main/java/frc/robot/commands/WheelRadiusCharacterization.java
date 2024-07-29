// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.subsystems.drive.DriveConstants.drivetrainConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * This class represents a command for characterizing the wheel radius of a swerve drive robot. It
 * calculates the effective wheel radius based on the gyro delta, drive base radius, and wheel
 * position delta. The command runs the robot at a constant velocity and measures the change in
 * wheel positions and gyro yaw. It then calculates the average wheel position change and uses it to
 * determine the effective wheel radius.
 */
public class WheelRadiusCharacterization extends Command {
  // Wheel Radius (meters) = Gyro Delta (radians) * Drive Base Radius (meters) / Wheel Position
  // Delta (radians)
  @AutoLogOutput private double gyroDelta = 0.0;
  @AutoLogOutput private double wheelPosDelta = 0.0;
  @AutoLogOutput private double currentEffectiveWheelRadius = 0.0;
  @AutoLogOutput private double lastGyroYawRads = 0.0;
  @AutoLogOutput private double accumGyroYawRads = 0.0;

  private double[] startWheelPositions;

  private Drive drive;
  private final DoubleSupplier gyroYawRadsSupplier = () -> drive.getRotation().getRadians();

  public WheelRadiusCharacterization(Drive drive) {
    this.drive = drive;
    addRequirements(this.drive);
  }

  @Override
  public void initialize() {
    lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
    accumGyroYawRads = 0.0;

    startWheelPositions = drive.getWheelRadiusCharacterizationPosition();

    System.out.println("\nStarted wheel radius characterization\n");
  }

  @Override
  public void execute() {
    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, .5));

    accumGyroYawRads += MathUtil.angleModulus(gyroYawRadsSupplier.getAsDouble() - lastGyroYawRads);
    lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();

    double averageWheelPosition = 0.0;
    double[] wheelPositions = drive.getWheelRadiusCharacterizationPosition();

    for (int i = 0; i < 4; i++) {
      averageWheelPosition += Math.abs(wheelPositions[i] - startWheelPositions[i]);
    }
    System.out.println(averageWheelPosition);
    // Check division by zero (if the robot doesn't move, the wheel positions will not change)
    currentEffectiveWheelRadius =
        (averageWheelPosition == 0.0)
            ? 0.0
            : (accumGyroYawRads * drivetrainConfig.driveBaseRadius())
                / (averageWheelPosition / 4.0);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    if (accumGyroYawRads <= Math.PI * 2.0) {
      System.out.println("\nNot enough data for characterization\n" + accumGyroYawRads);
    } else {
      System.out.println(
          "\nEffective Wheel Radius: "
              + Units.metersToInches(currentEffectiveWheelRadius)
              + " inches\n");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
