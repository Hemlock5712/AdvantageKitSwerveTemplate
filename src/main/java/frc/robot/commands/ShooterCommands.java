package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.function.DoubleSupplier;

public class ShooterCommands {
  private ShooterCommands() {}

  public static Command runSpeed(ShooterSubsystem shooter, DoubleSupplier speed) {
    return Commands.startEnd(
        () -> shooter.runVelocity(speed.getAsDouble()), shooter::stop, shooter);
  }
}
