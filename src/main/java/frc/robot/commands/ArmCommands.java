package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import java.util.function.DoubleSupplier;

public class ArmCommands {

  private ArmCommands() {}
  ;

  public static Command manualArmCommand(ArmSubsystem arm, DoubleSupplier supplier) {
    return Commands.run(
        () -> {
          arm.setVoltage(supplier.getAsDouble() * ArmConstants.MANUAL_ARM_MAX_VOLTS);
        },
        arm);
  }

  public static Command manualArmPos(ArmSubsystem arm, DoubleSupplier radianSupplier) {
    return Commands.runEnd(
        () -> {
          arm.setPositionRad(radianSupplier.getAsDouble());
        },
        arm::stop,
        arm);
  }

  public static Command autoArmToPosition(ArmSubsystem arm, double setpointInRadians) {
    return Commands.runOnce(
        () -> {
          arm.setPositionRad(setpointInRadians);
        });
  }
}
