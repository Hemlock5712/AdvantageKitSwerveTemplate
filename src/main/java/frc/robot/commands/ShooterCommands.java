package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.subsystems.ColorSensor.ColorSensor;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.function.DoubleSupplier;

public class ShooterCommands {
  private ShooterCommands() {}
  ;

  public static Command fullshot(
      ShooterSubsystem shooter, Intake intake, ColorSensor colorSensor, double velocityRadPerSec) {
    return new ConditionalCommand(
        scoreAmp(shooter)
            .raceWith(
                Commands.waitUntil(
                        () ->
                            MathUtil.isNear(
                                velocityRadPerSec,
                                shooter.getVelocityRadiansPerSec(),
                                ShooterConstants.VELOCITY_TOLERANCE.get()))
                    .andThen(
                        Commands.startEnd(
                            () -> intake.setVoltage(IntakeConstants.INTAKE_VOLTAGE.get()),
                            intake::stop,
                            intake))
                    .withTimeout(1)),
        Commands.none(),
        colorSensor::detectNote);
  }

  public static Command fullshotVoltage(
      ShooterSubsystem shooter, Intake intake, ColorSensor colorSensor) {
    return new ConditionalCommand(
        Commands.startEnd(() -> shooter.runVolts(5.0 * .99), shooter::stop, shooter)
            .raceWith(
                Commands.waitSeconds(.5)
                    .andThen(Commands.startEnd(() -> intake.setVoltage(10.0), intake::stop, intake))
                    .withTimeout(1.5)),
        Commands.none(),
        colorSensor::detectNote);
  }

  public static Command scoreAmp(ShooterSubsystem shooter) {
    return Commands.run(
        () ->
            shooter.runVelocity(
                Units.radiansPerSecondToRotationsPerMinute(
                    ShooterConstants.AMP_VELOCITY_RAD_PER_SEC.get())));
  }

  public static Command scoreSpeakerConstantSpeed(ShooterSubsystem shooter) {
    return Commands.run(
        () ->
            shooter.runVelocity(
                Units.radiansPerSecondToRotationsPerMinute(
                    ShooterConstants.SPEAKER_VELOCITY_RAD_PER_SEC.get())));
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
