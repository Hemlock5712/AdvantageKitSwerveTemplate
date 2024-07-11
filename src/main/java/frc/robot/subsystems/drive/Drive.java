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

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.VisionHelpers.TimestampedVisionUpdate;
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
  private final SysIdRoutine sysId;

  private static final double DEADBAND = 0.1;

  private static final double MAX_ROBOT_NOTE =
      4; // Max distance allowed from robot to note will be smaller in real life
  private static final double GP_ASSIST_kP = 5.0;

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(
          kinematics,
          rawGyroRotation,
          lastModulePositions,
          new Pose2d(),
          stateStdDevs,
          new Matrix<>(
              VecBuilder.fill(xyStdDevCoefficient, xyStdDevCoefficient, thetaStdDevCoefficient)));

  private SwerveDrivePoseEstimator odometryDrive =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

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
        this::setAutoStartPose,
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

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                state -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                voltage -> {
                  for (int i = 0; i < 4; i++) {
                    modules[i].runCharacterization(voltage.in(Volts));
                  }
                },
                null,
                this));
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
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
      odometryDrive.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
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

          Translation2d translationSpeeds =
              calulateToGamepieceNorm(
                  getPose().getTranslation(),
                  AllianceFlipUtil.apply(new Translation2d(3, 7)),
                  new Translation2d(robotRelativeXVel, robotRelativeYVel));

          ChassisSpeeds chassisSpeeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  translationSpeeds.getX(),
                  translationSpeeds.getY(),
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

  /**
   * Returns a command to run a quasistatic test in the specified direction.
   *
   * @param direction The direction to run the quasistatic test.
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /**
   * Returns a command to run a dynamic test in the specified direction.
   *
   * @param direction The direction to run the dynamic test.
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
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
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Drive")
  public Pose2d getDrive() {
    return odometryDrive.getEstimatedPosition();
  }

  /** Returns the current poseEstimator rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /**
   * Resets the current poseEstimator pose.
   *
   * @param pose The pose to reset to.
   */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /**
   * Resets the current odometry and poseEstimator pose.
   *
   * @param pose The pose to reset to.
   */
  public void setAutoStartPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    odometryDrive.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(
      Pose2d visionPose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(visionPose, timestamp, visionMeasurementStdDevs);
  }

  /**
   * Adds vision data to the pose esimation.
   *
   * @param visionData The vision data to add.
   */
  public void addVisionData(List<TimestampedVisionUpdate> visionData) {
    visionData.forEach(
        visionUpdate ->
            addVisionMeasurement(
                visionUpdate.pose(), visionUpdate.timestamp(), visionUpdate.stdDevs()));
  }

  private double normRotations(double rotations) {
    return MathUtil.inputModulus(rotations, 0.0, 1.0);
  }

  public Translation2d calulateToGamepieceNorm(
      Translation2d robotPose, Translation2d gamepiece, Translation2d velocity) {
    if (velocity.getNorm() == 0) {
      return velocity;
    }

    Translation2d robotToNote = gamepiece.minus(robotPose);
    double projFactor =
        ((robotToNote.getX() * velocity.getX()) + (robotToNote.getY() * velocity.getY()))
            / ((velocity.getX() * velocity.getX()) + (velocity.getY() * velocity.getY()));
    Translation2d projRobotToNote = velocity.times(projFactor);
    Translation2d perpRobotToNote = robotToNote.minus(projRobotToNote);

    Logger.recordOutput(
        "Custom 3",
        (MathUtil.inputModulus(
            robotToNote.getAngle().getRotations() - velocity.getAngle().getRotations(),
            -0.5,
            0.5)));

    Logger.recordOutput("Custom 4", gamepiece);

    Logger.recordOutput(
        "Custom 2",
        new Translation2d(robotPose.getX() + velocity.getX(), robotPose.getY() + velocity.getY()));

    double angleToGamePiece =
        Math.abs(
            MathUtil.inputModulus(
                robotToNote.getAngle().getRotations() - velocity.getAngle().getRotations(),
                -0.5,
                0.5));

    if ((AllianceFlipUtil.shouldFlip() && angleToGamePiece < 0.4)
        || (!AllianceFlipUtil.shouldFlip() && angleToGamePiece < 0.1)
            && robotToNote.getNorm() < MAX_ROBOT_NOTE) {
      Translation2d assistVelocity = perpRobotToNote.times(GP_ASSIST_kP);
      Translation2d newVelocity = velocity.plus(assistVelocity);
      Translation2d normVelocity = newVelocity.div(newVelocity.getNorm());
      Translation2d finalVelocity = normVelocity.times(velocity.getNorm());

      Logger.recordOutput(
          "Custom",
          new Translation2d(
              robotPose.getX() + finalVelocity.getX(), robotPose.getY() + finalVelocity.getY()));

      return finalVelocity;
    } else {
      return velocity;
    }
  }
}
