package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final SimpleMotorFeedforward feedForward;

  public ShooterSubsystem(ShooterIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        feedForward =
            new SimpleMotorFeedforward(
                ShooterConstants.Real.FeedForwardConstants.kS,
                ShooterConstants.Real.FeedForwardConstants.kV);
        io.configurePID(
            ShooterConstants.Real.PIDConstants.kP,
            ShooterConstants.Real.PIDConstants.kI,
            ShooterConstants.Real.PIDConstants.kD);
        break;
      case SIM:
        feedForward =
            new SimpleMotorFeedforward(
                ShooterConstants.Sim.FeedForwardConstants.kS,
                ShooterConstants.Sim.FeedForwardConstants.kV);
        io.configurePID(
            ShooterConstants.Sim.PIDConstants.kP,
            ShooterConstants.Sim.PIDConstants.kI,
            ShooterConstants.Sim.PIDConstants.kD);
        break;
      default:
        feedForward = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public void runVelocity(double velocityRPM) {
    double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocity(velocityRadPerSec, feedForward.calculate(velocityRadPerSec));

    Logger.recordOutput("Shooter Setpoint RPM", velocityRPM);
  }

  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  public void stop() {
    io.stop();
  }

  @AutoLogOutput
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    io.setVoltage(volts);
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty(
        "Shooter RPM",
        () -> Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec),
        null);
  }
}
