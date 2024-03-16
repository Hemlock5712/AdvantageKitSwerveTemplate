package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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

    topFeedForward =
        new SimpleMotorFeedforward(
            ShooterConstants.CURRENT_TUNE.top().ks(), ShooterConstants.CURRENT_TUNE.top().kv());
    topIO.configurePID(ShooterConstants.FlywheelModelConstants.Top.kP.get(), 0, 0);
    bottomFeedForward =
        new SimpleMotorFeedforward(
            ShooterConstants.CURRENT_TUNE.bottom().ks(),
            ShooterConstants.CURRENT_TUNE.bottom().kv());
    bottomIO.configurePID(ShooterConstants.FlywheelModelConstants.Bottom.kP.get(), 0, 0);

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
    if (ShooterConstants.FlywheelModelConstants.Top.kP.hasChanged(0)) {
      topIO.configurePID(ShooterConstants.FlywheelModelConstants.Top.kP.get(), 0, 0);
    }
    if (ShooterConstants.FlywheelModelConstants.Bottom.kP.hasChanged(0)) {
      bottomIO.configurePID(ShooterConstants.FlywheelModelConstants.Bottom.kP.get(), 0, 0);
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
