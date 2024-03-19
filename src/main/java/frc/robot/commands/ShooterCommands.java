package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.ShooterStateHelpers;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ShooterCommands {
  private ShooterCommands() {}

  public static Command runSpeed(ShooterSubsystem shooter, DoubleSupplier speed) {
    return Commands.startEnd(
        () -> shooter.runVelocity(speed.getAsDouble()), shooter::stop, shooter);
  }

  /** assumes the shooter is at the correct speed and the arm is in the correct position */
  public static Command autoShoot(
      ShooterStateHelpers shooterStateHelpers,
      Intake intake,
      BooleanSupplier hasNote,
      ArmSubsystem arm) {
    return shooterStateHelpers
        .waitUntilCanShootAuto()
        .andThen(
            Commands.runOnce(() -> intake.setVoltage(IntakeConstants.INTAKE_VOLTAGE.get()), intake))
        .andThen(Commands.waitUntil(() -> !hasNote.getAsBoolean()).withTimeout(1))
        .andThen(Commands.waitSeconds(.3))
        .andThen(Commands.runOnce(() -> intake.setVoltage(0), intake))
        .andThen(ArmCommands.autoArmToPosition(arm, ArmConstants.Positions.INTAKE_POS_RAD::get));
  }
}
