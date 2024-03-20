package frc.robot.subsystems.shooter;

import com.revrobotics.*;

public class ShooterIOSim implements ShooterIO {
  private double targetVelocity = 0;

  public ShooterIOSim() {}

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.velocityRadPerSec = targetVelocity;
  }

  @Override
  public void setVoltage(double volts) {
    targetVelocity = volts * 30;
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double feedForwardVolts) {
    targetVelocity = velocityRadPerSec;
  }

  @Override
  public void stop() {
    targetVelocity = 0;
  }
}
