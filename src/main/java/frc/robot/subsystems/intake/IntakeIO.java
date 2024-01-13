package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
    public double temperature = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run open loop at the specified voltage/ */
  public default void setVoltage(double volts) {}

  /** Run closed loop at the specified velocity/ */
  public default void setVelocity(double velocityRadPerSec, double feedForwardVolts) {}

  /** Stop in open loop. */
  public default void stop() {}

  /** Set velocity PID constants/ */
  public default void configurePID(double kP, double kI, double kD) {}

  public default void runVolts(double volts) {}
}
