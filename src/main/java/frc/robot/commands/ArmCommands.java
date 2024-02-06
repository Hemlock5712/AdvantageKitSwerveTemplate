package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import java.util.function.DoubleSupplier;

public class ArmCommands {

  private ArmCommands() {}
  ;

  public static Command manuelArmCommand(ArmSubsystem arm, DoubleSupplier supplier) {
    return Commands.run(
        () -> {
          arm.setVoltage(supplier.getAsDouble() * ArmConstants.MANUEL_ARM_MAX_VOLTS);
        },
        arm);
  }
}
