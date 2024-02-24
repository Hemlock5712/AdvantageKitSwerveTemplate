package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
    public double motorTemperatureCelsius = 0.0;
    public boolean motorSensorFault = false;
    public boolean motorBrownOut = false;
    public int motorCANID = -1;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setVelocity(double velocityRadPerSec, double feedForwardVolts) {}

  public default void stop() {}

  public default void configurePID(double kP, double kI, double kD) {}
}
