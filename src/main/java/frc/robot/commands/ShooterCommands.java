package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ColorSensor.ColorSensor;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.function.DoubleSupplier;

public class ShooterCommands {
  private ShooterCommands() {}
  ;

  public static Command fullshot(ShooterSubsystem shooter, Intake intake, ColorSensor colorSensor) {
    if (!colorSensor.detectNote()) {
      return Commands.none();
    }
    return scoreAmp(shooter).raceWith(
      Commands.waitUntil(() -> MathUtil.isNear(
        ShooterConstants.AMP_VELOCITY_RAD_PER_SEC,
        shooter.getVelocityRPM(),
        0.5))
      .andThen(Commands.startEnd(
          () -> intake.setVoltage(IntakeConstants.INTAKE_VOLTAGE),
          intake::stop,
          intake)
      ).withTimeout(1)
    );
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

  public static Command runVoltage(ShooterSubsystem shooter) {
    return Commands.run(() -> shooter.runVolts(12.0 * 0.5));
  }
}
