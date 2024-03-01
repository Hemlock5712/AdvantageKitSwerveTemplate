package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterConstants.Real.PIDConstants.BottomConstants;
import frc.robot.subsystems.shooter.ShooterConstants.Real.PIDConstants.TopConstants;
import frc.robot.util.ErrorChecker;
import lombok.Getter;
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
  @AutoLogOutput @Getter private double targetVelocityRadPerSec;

  public ShooterSubsystem(ShooterIO topIO, ShooterIO bottomIO) {
    this.topIO = topIO;
    this.bottomIO = bottomIO;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.getMode()) {
      case REAL, REPLAY -> {
        topFeedForward =
            new SimpleMotorFeedforward(
                ShooterConstants.Real.FeedForwardConstants.TopConstants.kS.get(),
                ShooterConstants.Real.FeedForwardConstants.TopConstants.kV.get());
        topIO.configurePID(TopConstants.kP.get(), TopConstants.kI.get(), TopConstants.kD.get());
        bottomFeedForward =
            new SimpleMotorFeedforward(
                ShooterConstants.Real.FeedForwardConstants.BottomConstants.kS.get(),
                ShooterConstants.Real.FeedForwardConstants.BottomConstants.kV.get());
        bottomIO.configurePID(
            BottomConstants.kP.get(), BottomConstants.kI.get(), BottomConstants.kD.get());
      }
      case SIM -> {
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
      }
      default -> {
        topFeedForward = new SimpleMotorFeedforward(0.0, 0.0);
        bottomFeedForward = new SimpleMotorFeedforward(0.0, 0.0);
      }
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
    ErrorChecker.checkError(topInputs);
    ErrorChecker.checkError(bottomInputs);
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

    Logger.recordOutput("ShooterSubsystem/Setpoint Rad per s", velocityRadPerSec);
    Logger.recordOutput(
        "ShooterSubsystem/attempt ff top volts", topFeedForward.calculate(velocityRadPerSec));
    Logger.recordOutput("ShooterSubsystem/attempt volts", -1.);
  }

  public void runVolts(double volts) {
    Logger.recordOutput("ShooterSubsystem/Setpoint Rad per s", 0.);
    Logger.recordOutput("ShooterSubsystem/attempt volts", volts);
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
    Logger.recordOutput(
        "shooter top offset", topInputs.velocityRadPerSec - targetVelocityRadPerSec);
    return Math.abs(topInputs.velocityRadPerSec - targetVelocityRadPerSec)
        < ShooterConstants.VELOCITY_TOLERANCE.get();
  }

  @AutoLogOutput
  public boolean bottomShooterNearTargetVelocity() {
    Logger.recordOutput(
        "shooter bottom offset", bottomInputs.velocityRadPerSec - targetVelocityRadPerSec);
    return Math.abs(bottomInputs.velocityRadPerSec - targetVelocityRadPerSec)
        < ShooterConstants.VELOCITY_TOLERANCE.get();
  }

  public boolean bothShootersNearTargetVelocity() {
    return topShooterNearTargetVelocity() && bottomShooterNearTargetVelocity();
  }
}
