package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;

// TODO: Convert this to use the navx
public class GyrioIONavX2 implements GyroIO {
  private final AHRS navx = new AHRS(SPI.Port.kMXP);

  public GyrioIONavX2() {

    // TODO: What else is missin ghere
    navx.reset();
    navx.zeroYaw();

    //ORIGINAL PIGEON CODE: TODO: REMOVE ONCE VERIFIED
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
    inputs.yawVelocityRadPerSec = navx.getRate();

    //TODO: REMOVE ORIGINAL CODE BELOW ONCE VERIFIED
    /*
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
    */
  }
}
