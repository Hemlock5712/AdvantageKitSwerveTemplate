package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import java.util.OptionalDouble;
import java.util.Queue;

public class GyroIONavX2 implements GyroIO {
  private final AHRS navx = new AHRS(SPI.Port.kMXP);
  private final Queue<Double> yawPositionQueue;

  public GyroIONavX2() {
    navx.reset();
    navx.resetDisplacement();
    navx.zeroYaw();

    yawPositionQueue =
        SparkMaxOdometryThread.getInstance()
            .registerSignal(
                () -> {
                  boolean valid = navx.isConnected();
                  if (valid) {
                    return OptionalDouble.of(navx.getYaw());
                  } else {
                    return OptionalDouble.empty();
                  }
                });
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = navx.isConnected();
    inputs.yawPosition = Rotation2d.fromDegrees(-navx.getYaw());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(-navx.getRawGyroZ());
    inputs.odometryYawPositions =
        yawPositionQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);

    yawPositionQueue.clear();
  }

  @Override
  public void resetGyro() {
    navx.reset();
  }
}
