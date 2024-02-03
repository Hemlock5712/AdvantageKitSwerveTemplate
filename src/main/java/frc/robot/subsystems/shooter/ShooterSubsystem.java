package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
  private final ShooterIO topIO;
  private final ShooterIO bottomIO;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final SimpleMotorFeedforward topFeedForward;
  private final SimpleMotorFeedforward bottomFeedForward;

  public ShooterSubsystem(ShooterIO topIO, ShooterIO bottomIO) {
    this.topIO = topIO;
    this.bottomIO = bottomIO;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.getMode()) {
      case REAL:
      case REPLAY:
        topFeedForward =
            new SimpleMotorFeedforward(
                ShooterConstants.Real.FeedForwardConstants.TopConstants.kS,
                ShooterConstants.Real.FeedForwardConstants.TopConstants.kV);
        topIO.configurePID(
            ShooterConstants.Real.PIDConstants.TopConstants.kP,
            ShooterConstants.Real.PIDConstants.TopConstants.kI,
            ShooterConstants.Real.PIDConstants.TopConstants.kD);

        bottomFeedForward =
                new SimpleMotorFeedforward(
                        ShooterConstants.Real.FeedForwardConstants.TopConstants.kS,
                        ShooterConstants.Real.FeedForwardConstants.TopConstants.kV);
        bottomIO.configurePID(
                ShooterConstants.Real.PIDConstants.TopConstants.kP,
                ShooterConstants.Real.PIDConstants.TopConstants.kI,
                ShooterConstants.Real.PIDConstants.TopConstants.kD);
        break;
      case SIM:
        topFeedForward =
                new SimpleMotorFeedforward(
                        ShooterConstants.Sim.FeedForwardConstants.TopConstants.kS,
                        ShooterConstants.Sim.FeedForwardConstants.TopConstants.kV);
        topIO.configurePID(
                ShooterConstants.Sim.PIDConstants.TopConstants.kP,
                ShooterConstants.Sim.PIDConstants.TopConstants.kI,
                ShooterConstants.Sim.PIDConstants.TopConstants.kD);

        bottomFeedForward =
                new SimpleMotorFeedforward(
                        ShooterConstants.Sim.FeedForwardConstants.TopConstants.kS,
                        ShooterConstants.Sim.FeedForwardConstants.TopConstants.kV);
        bottomIO.configurePID(
                ShooterConstants.Sim.PIDConstants.TopConstants.kP,
                ShooterConstants.Sim.PIDConstants.TopConstants.kI,
                ShooterConstants.Sim.PIDConstants.TopConstants.kD);
        break;
      default:
        topFeedForward = new SimpleMotorFeedforward(0.0, 0.0);
        bottomFeedForward = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }
  }

  @Override
  public void periodic() {
    //I'm not sure if this updates the inputs correctly, since the two ios are
    //using the same inputs
    topIO.updateInputs(inputs);
    bottomIO.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public void runVelocity(double velocityRPM) {
    double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    topIO.setVelocity(velocityRadPerSec, topFeedForward.calculate(velocityRadPerSec));
    bottomIO.setVelocity(velocityRadPerSec, bottomFeedForward.calculate(velocityRadPerSec));

    Logger.recordOutput("Shooter Setpoint RPM", velocityRPM);
  }

  public void runVolts(double volts) {
    topIO.setVoltage(volts);
    bottomIO.setVoltage(volts);
  }

  public void stop() {
    topIO.stop();
    bottomIO.stop();
  }

  @AutoLogOutput
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  /** Runs forwards at the commanded voltage. */
  //Change this to bottomIO when characterizing the bottom wheels
  public void runCharacterizationVolts(double volts) {
    topIO.setVoltage(volts);
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
