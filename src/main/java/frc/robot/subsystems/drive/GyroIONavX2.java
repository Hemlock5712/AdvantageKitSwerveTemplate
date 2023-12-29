package frc.robot.subsystems.drive;

import java.util.Queue;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;

// TODO: Convert this to use the navx
public class GyroIONavX2 implements GyroIO {
  private final AHRS navx = new AHRS(SPI.Port.kMXP);
  private final Queue<Double> yawPositionQueue;

  public GyroIONavX2() {

    // TODO: What else is missing here
    navx.reset();
    navx.resetDisplacement();
    navx.zeroYaw();
    yawPositionQueue = SparkMaxOdometryThread.getInstance().registerSignal(navx::getYaw);

    // ORIGINAL PIGEON CODE: TODO: REMOVE ONCE VERIFIED
    /*
         *     pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(100.0);
    yawVelocity.setUpdateFrequency(100.0);
    pigeon.optimizeBusUtilization();
         */
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = navx.isConnected();
    inputs.yawPosition = Rotation2d.fromDegrees(navx.getYaw());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(navx.getRawGyroZ());
    inputs.odometryYawPositions = yawPositionQueue.stream().map((Double value) -> Rotation2d.fromDegrees(value)).toArray(Rotation2d[]::new);

    yawPositionQueue.clear();
    // TODO: REMOVE ORIGINAL CODE BELOW ONCE VERIFIED
    /*
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
    */
  }
}
