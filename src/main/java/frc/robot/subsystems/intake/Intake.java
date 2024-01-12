package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final SimpleMotorFeedforward feedForward;

  public Intake(IntakeIO io) {
    this.io = io;

    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        feedForward =
            new SimpleMotorFeedforward(
                IntakeConstants.Real.FeedForwardConstants.kS,
                IntakeConstants.Real.FeedForwardConstants.kV);
        io.configurePID(
            IntakeConstants.Real.PIDConstants.kP,
            IntakeConstants.Real.PIDConstants.kI,
            IntakeConstants.Real.PIDConstants.kD);
        break;
      case SIM:
        feedForward =
            new SimpleMotorFeedforward(
                IntakeConstants.Sim.FeedForwardConstants.kS,
                IntakeConstants.Sim.FeedForwardConstants.kV);
        io.configurePID(
            IntakeConstants.Sim.PIDConstants.kP,
            IntakeConstants.Sim.PIDConstants.kI,
            IntakeConstants.Sim.PIDConstants.kD);
        break;
      default:
        feedForward = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public void runVelocity(double velocityRPM) {
    double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocity(velocityRadPerSec, feedForward.calculate(velocityRadPerSec));
  }

  public void stop() {
    io.stop();
  }

  @AutoLogOutput
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  public void runCharacterizationVolts(double volts) {
    io.setVoltage(volts);
  }

  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }
}
