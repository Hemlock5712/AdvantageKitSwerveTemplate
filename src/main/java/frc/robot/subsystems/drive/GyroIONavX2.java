package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;

public class GyroIONavX2 implements GyroIO {
  private final AHRS navx = new AHRS(SPI.Port.kMXP);

  public GyroIONavX2() {
    navx.reset();
    navx.resetDisplacement();
    navx.zeroYaw();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = navx.isConnected();
    inputs.yawPosition = Rotation2d.fromDegrees(navx.getYaw());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(navx.getRawGyroZ());
  }
}
