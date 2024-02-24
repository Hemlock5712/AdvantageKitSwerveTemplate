package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterConstants.Real.PIDConstants.BottomConstants;
import frc.robot.subsystems.shooter.ShooterConstants.Real.PIDConstants.TopConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
  private final ShooterIO topIO;
  private final ShooterIO bottomIO;
  private final ShooterIOInputsAutoLogged topInputs = new ShooterIOInputsAutoLogged();
  private final ShooterIOInputsAutoLogged bottomInputs = new ShooterIOInputsAutoLogged();
  private final SimpleMotorFeedforward topFeedForward;
  private final SimpleMotorFeedforward bottomFeedForward;
  public final SysIdRoutine sysid;
  private double targetVelocityRadPerSec;

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
                ShooterConstants.Real.FeedForwardConstants.TopConstants.kS.get(),
                ShooterConstants.Real.FeedForwardConstants.TopConstants.kV.get());
        topIO.configurePID(
            ShooterConstants.Real.PIDConstants.TopConstants.kP.get(),
            ShooterConstants.Real.PIDConstants.TopConstants.kI.get(),
            ShooterConstants.Real.PIDConstants.TopConstants.kD.get());

        bottomFeedForward =
            new SimpleMotorFeedforward(
                ShooterConstants.Real.FeedForwardConstants.BottomConstants.kS.get(),
                ShooterConstants.Real.FeedForwardConstants.BottomConstants.kV.get());
        bottomIO.configurePID(
            ShooterConstants.Real.PIDConstants.BottomConstants.kP.get(),
            ShooterConstants.Real.PIDConstants.BottomConstants.kI.get(),
            ShooterConstants.Real.PIDConstants.BottomConstants.kD.get());
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
                ShooterConstants.Sim.FeedForwardConstants.BottomConstants.kS,
                ShooterConstants.Sim.FeedForwardConstants.BottomConstants.kV);
        bottomIO.configurePID(
            ShooterConstants.Sim.PIDConstants.BottomConstants.kP,
            ShooterConstants.Sim.PIDConstants.BottomConstants.kI,
            ShooterConstants.Sim.PIDConstants.BottomConstants.kD);
        break;
      default:
        topFeedForward = new SimpleMotorFeedforward(0.0, 0.0);
        bottomFeedForward = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }

    sysid =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                state -> Logger.recordOutput("ShooterSubsystem/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                voltage -> {
                  runVolts(voltage.in(Volts));
                },
                null,
                this));

    targetVelocityRadPerSec = 0.0;
  }

  @Override
  public void periodic() {
    // I'm not sure if this updates the inputs correctly, since the two ios are
    // using the same inputs
    topIO.updateInputs(topInputs);
    bottomIO.updateInputs(bottomInputs);
    Logger.processInputs("ShooterSubsystem/Top", topInputs);
    Logger.processInputs("ShooterSubsystem/Bottom", bottomInputs);

    updateControlConstants();
  }

  private void updateControlConstants() {
    if (TopConstants.kP.hasChanged(0)
        || TopConstants.kI.hasChanged(0)
        || TopConstants.kD.hasChanged(0)) {
      topIO.configurePID(TopConstants.kP.get(), TopConstants.kI.get(), TopConstants.kD.get());
    }
    if (BottomConstants.kP.hasChanged(0)
        || BottomConstants.kI.hasChanged(0)
        || BottomConstants.kD.hasChanged(0)) {
      bottomIO.configurePID(
          BottomConstants.kP.get(), BottomConstants.kI.get(), BottomConstants.kD.get());
    }
  }

  public void runVelocity(double velocityRadPerSec) {
    targetVelocityRadPerSec = velocityRadPerSec;
    topIO.setVelocity(velocityRadPerSec, topFeedForward.calculate(velocityRadPerSec));
    bottomIO.setVelocity(velocityRadPerSec, bottomFeedForward.calculate(velocityRadPerSec));

    Logger.recordOutput("Shooter Setpoint Rad/s", velocityRadPerSec);
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
  public double getAverageVelocityRadiansPerSec() {
    return (topInputs.velocityRadPerSec + bottomInputs.velocityRadPerSec) / 2.0;
  }

  @AutoLogOutput
  public boolean topShooterNearTargetVelocity() {
    return Math.abs(topInputs.velocityRadPerSec - targetVelocityRadPerSec)
        < ShooterConstants.VELOCITY_TOLERANCE.get();
  }

  @AutoLogOutput
  public boolean bottomShooterNearTargetVelocity() {
    return Math.abs(topInputs.velocityRadPerSec - targetVelocityRadPerSec)
        < ShooterConstants.VELOCITY_TOLERANCE.get();
  }
}
