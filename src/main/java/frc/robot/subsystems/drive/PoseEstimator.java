package frc.robot.subsystems.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.NoSuchElementException;
import java.util.Optional;

public class PoseEstimator {
  private static final double poseBufferSizeSeconds = 1.0;

  private final TimeInterpolatableBuffer<Pose2d> poseBuffer;
  private final Matrix<N3, N1> odometryStdDevs;
  private final SwerveDriveKinematics kinematics;

  private SwerveDriveWheelPositions lastWheelPositions =
      new SwerveDriveWheelPositions(
          new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
          });
  private Rotation2d lastGyroRotation = new Rotation2d();
  private Pose2d odometryPose = new Pose2d();
  private Pose2d estimatedPose = new Pose2d();

  public PoseEstimator(Matrix<N3, N1> odometryStdDevs, SwerveDriveKinematics kinematics) {
    this.poseBuffer = TimeInterpolatableBuffer.createBuffer(poseBufferSizeSeconds);
    this.odometryStdDevs = odometryStdDevs;
    this.kinematics = kinematics;
  }

  public void addOdometryObservation(OdometryObservation observation) {
    Twist2d twist = kinematics.toTwist2d(lastWheelPositions, observation.wheelPositions());
    lastWheelPositions = observation.wheelPositions();
    // Check gyro connected
    if (observation.gyroRotation != null) {
      // Update dtheta for twist if gyro connected
      twist =
          new Twist2d(
              twist.dx, twist.dy, observation.gyroRotation.minus(lastGyroRotation).getRadians());
      lastGyroRotation = observation.gyroRotation;
    }
    // Add twist to odometry pose
    odometryPose = odometryPose.exp(twist);
    // Add pose to buffer at timestamp
    poseBuffer.addSample(observation.timestamp(), odometryPose);
    // Calculate diff from last odometry pose and add onto pose estimate
    estimatedPose = estimatedPose.exp(twist);
  }

  public void addVisionObservation(VisionObservation observation) {
    try {
      if (poseBuffer.getInternalBuffer().lastKey() - poseBufferSizeSeconds
          > observation.timestamp()) {
        return;
      }
    } catch (NoSuchElementException ex) {
      return;
    }
    // Get odometry based pose at timestamp
    var sample = poseBuffer.getSample(observation.timestamp());
    if (sample.isEmpty()) {
      // exit if not there
      return;
    }

    // sample --> odometryPose transform and backwards of that
    var sampleToOdometryTransform = new Transform2d(sample.get(), odometryPose);
    var odometryToSampleTransform = new Transform2d(odometryPose, sample.get());
    // get old estimate by applying odometryToSample Transform
    Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);

    // Calculate 3 x 3 vision matrix
    var r = new double[3];
    for (int i = 0; i < 3; ++i) {
      r[i] = observation.stdDevs().get(i, 0) * observation.stdDevs().get(i, 0);
    }
    // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
    // and C = I. See wpimath/algorithms.md.
    Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
    for (int row = 0; row < 3; ++row) {
      double stdDev = odometryStdDevs.get(row, 0);
      if (stdDev == 0.0) {
        visionK.set(row, row, 0.0);
      } else {
        visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
      }
    }
    // difference between estimate and vision pose
    Transform2d transform = new Transform2d(estimateAtTime, observation.visionPose());
    // scale transform by visionK
    var kTimesTransform =
        visionK.times(
            VecBuilder.fill(
                transform.getX(), transform.getY(), transform.getRotation().getRadians()));
    Transform2d scaledTransform =
        new Transform2d(
            kTimesTransform.get(0, 0),
            kTimesTransform.get(1, 0),
            Rotation2d.fromRadians(kTimesTransform.get(2, 0)));

    // Recalculate current estimate by applying scaled transform to old estimate
    // then replaying odometry data
    estimatedPose = estimateAtTime.plus(scaledTransform).plus(sampleToOdometryTransform);
  }

  public void resetPose(Pose2d pose) {
    estimatedPose = pose;
    odometryPose = pose;
    poseBuffer.clear();
  }

  public Pose2d getOdomPose() {
    return odometryPose;
  }

  public Pose2d getEstimatedPose() {
    return estimatedPose;
  }

  public Optional<Pose2d> sampleAt(double timestamp) {
    return poseBuffer
        .getSample(timestamp)
        .map(sample -> sample.plus(new Transform2d(odometryPose, estimatedPose)));
  }

  public record OdometryObservation(
      SwerveDriveWheelPositions wheelPositions, Rotation2d gyroRotation, double timestamp) {}

  public record VisionObservation(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {}
}
