// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.PoseEstimator.OdometryObservation;
import frc.robot.subsystems.drive.PoseEstimator.VisionObservation;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR

  private double filteredX = 0;
  private double filteredY = 0;
  private final LinearFilter xFilter = LinearFilter.movingAverage(10);
  private final LinearFilter yFilter = LinearFilter.movingAverage(10);

  private static final double DEADBAND = 0.1;

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
  private Rotation2d rawGyroRotation = new Rotation2d();
  private double yawVelocityRadPerSec = 0;
  private double yawTimeStamp = 0;

  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private final PoseEstimator poseEstimator;

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Start threads (no-op for each if no signals have been created)
    PhoenixOdometryThread.getInstance().start();
    SparkMaxOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        () -> kinematics.toChassisSpeeds(getModuleStates()),
        this::runVelocity,
        new HolonomicPathFollowerConfig(
            new PIDConstants(
                PPtranslationConstants.kP, PPtranslationConstants.kI, PPtranslationConstants.kD),
            new PIDConstants(
                PProtationConstants.kP, PProtationConstants.kI, PProtationConstants.kD),
            drivetrainConfig.maxLinearVelocity(),
            drivetrainConfig.driveBaseRadius(),
            new ReplanningConfig()),
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
        this);
    PathPlannerLogging.setLogActivePathCallback(
        activePath ->
            Logger.recordOutput(
                "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()])));
    PathPlannerLogging.setLogTargetPoseCallback(
        targetPose -> Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose));

    poseEstimator = new PoseEstimator(DriveConstants.stateStdDevs, DriveConstants.kinematics);
    // PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    for (var module : modules) {
      module.updateInputs();
    }
    odometryLock.unlock();
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
        yawVelocityRadPerSec = gyroInputs.yawVelocityRadPerSec;
        yawTimeStamp = gyroInputs.odometryYawTimestamps[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
        yawTimeStamp = sampleTimestamps[i];
      }

      // Apply update
      poseEstimator.addOdometryObservation(
          new OdometryObservation(
              new SwerveDriveWheelPositions(modulePositions), rawGyroRotation, yawTimeStamp));

      ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(getModuleStates());
      Translation2d rawFieldRelativeVelocity =
          new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
              .rotateBy(getRotation());

      filteredX = xFilter.calculate(rawFieldRelativeVelocity.getX());
      filteredY = yFilter.calculate(rawFieldRelativeVelocity.getY());
    }
  }

  public Command swerveDriveCommand(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // Get robot relative vel
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;

          double robotRelativeXVel = linearVelocity.getX() * drivetrainConfig.maxLinearVelocity();
          double robotRelativeYVel = linearVelocity.getY() * drivetrainConfig.maxLinearVelocity();

          // Translation2d translationSpeeds =
          //     calulateToGamepieceNorm(
          //         AllianceFlipUtil.apply(new Translation2d(3, 7)),
          //         new Translation2d(robotRelativeXVel, robotRelativeYVel));

          ChassisSpeeds chassisSpeeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  robotRelativeXVel,
                  robotRelativeYVel,
                  omega * drivetrainConfig.maxAngularVelocity(),
                  isFlipped
                      ? this.getRotation().plus(new Rotation2d(Math.PI))
                      : this.getRotation());

          this.runVelocity(chassisSpeeds);
        },
        this);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    setModuleStates(setpointStates);
  }

  public void setModuleStates(SwerveModuleState[] setpointStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates, drivetrainConfig.maxLinearVelocity());

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = moduleTranslations[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the current pose estimation. */
  @AutoLogOutput(key = "Odometry/PoseEstimation")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPose();
  }

  /** Returns the current poseEstimator rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  @AutoLogOutput(key = "Drive/GyroRate")
  public double gyroRateDegrees() {
    return Units.radiansToDegrees(yawVelocityRadPerSec);
  }

  @AutoLogOutput(key = "Drive/GyroTimeStamp")
  public double gyroTimeStamp() {
    return yawTimeStamp;
  }

  /**
   * Resets the current poseEstimator pose.
   *
   * @param pose The pose to reset to.
   */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPose(pose);
  }

  /**
   * Adds vision data to the pose esimation.
   *
   * @param visionData The vision data to add.
   */
  public void addVisionData(List<VisionObservation> visionData) {
    visionData.stream()
        .sorted(Comparator.comparingDouble(VisionObservation::timestamp))
        .forEach(poseEstimator::addVisionObservation);
  }

  @AutoLogOutput
  public Translation2d getFieldRelativeVelocity() {
    return new Translation2d(filteredX, filteredY);
  }

  public double[] getWheelRadiusCharacterizationPosition() {
    return Arrays.stream(modules).mapToDouble(Module::getPositionRadians).toArray();
  }

  public void setWheelsToCircle() {
    Rotation2d[] turnAngles =
        Arrays.stream(DriveConstants.moduleTranslations)
            .map(translation -> translation.getAngle().plus(new Rotation2d(Math.PI / 2.0)))
            .toArray(Rotation2d[]::new);
    SwerveModuleState[] desiredStates =
        Arrays.stream(turnAngles)
            .map(angle -> new SwerveModuleState(0, angle))
            .toArray(SwerveModuleState[]::new);
    setModuleStates(desiredStates);
  }
}
