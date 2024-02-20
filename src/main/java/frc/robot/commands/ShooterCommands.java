package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.subsystems.beamBreak.BeamBreak;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.function.DoubleSupplier;

public class ShooterCommands {
  private ShooterCommands() {}
  ;

  public static Command fullshot(
      ShooterSubsystem shooter, Intake intake, BeamBreak beamBreak, double velocityRadPerSec) {
    return new ConditionalCommand(
        scoreAmp(shooter)
            .raceWith(
                Commands.waitUntil(
                        () ->
                            MathUtil.isNear(
                                velocityRadPerSec,
                                shooter.getVelocityRadiansPerSec(),
                                ShooterConstants.VELOCITY_TOLERANCE))
                    .andThen(
                        Commands.startEnd(
                            () -> intake.setVoltage(IntakeConstants.INTAKE_VOLTAGE),
                            intake::stop,
                            intake))
                    .withTimeout(1)),
        Commands.none(),
        beamBreak::detectNote);
  }

  public static Command fullshotVoltage(
      ShooterSubsystem shooter, Intake intake, BeamBreak beamBreak) {
    return new ConditionalCommand(
        Commands.startEnd(() -> shooter.runVolts(5.0 * .99), shooter::stop, shooter)
            .raceWith(
                Commands.waitSeconds(.5)
                    .andThen(Commands.startEnd(() -> intake.setVoltage(10.0), intake::stop, intake))
                    .withTimeout(1.5)),
        Commands.none(),
        beamBreak::detectNote);
  }

  public static Command scoreAmp(ShooterSubsystem shooter) {
    return Commands.run(
        () ->
            shooter.runVelocity(
                Units.radiansPerSecondToRotationsPerMinute(
                    ShooterConstants.AMP_VELOCITY_RAD_PER_SEC)));
  }

  public static Command scoreSpeakerConstantSpeed(ShooterSubsystem shooter) {
    return Commands.run(
        () ->
            shooter.runVelocity(
                Units.radiansPerSecondToRotationsPerMinute(
                    ShooterConstants.SPEAKER_VELOCITY_RAD_PER_SEC)));
  }

  public static Command scoreSpeakerVariableSpeed(
      ShooterSubsystem shooter, DoubleSupplier velocityRadPerSecSupplier) {
    return Commands.run(
        () ->
            shooter.runVelocity(
                Units.radiansPerSecondToRotationsPerMinute(
                    velocityRadPerSecSupplier.getAsDouble())));
  }
}
