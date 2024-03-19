package frc.robot.util;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleConsumer;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class SysIdBuilder {
  private final LoggedDashboardChooser<Command> autoChooser;

  public SysIdBuilder(LoggedDashboardChooser<Command> autoChooser) {
    this.autoChooser = autoChooser;
  }

  public void createSysId(Subsystem subsystem, DoubleConsumer runVolts) {
    final SysIdRoutine sysId = getSysIdRoutine(subsystem, runVolts);

    final String baseName = subsystem.getName() + " SysId ";

    autoChooser.addOption(
        baseName + "quasistatic forward", sysId.quasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        baseName + "quasistatic reverse", sysId.quasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        baseName + "dynamic forward", sysId.dynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        baseName + "dynamic reverse", sysId.dynamic(SysIdRoutine.Direction.kReverse));
  }

  private static SysIdRoutine getSysIdRoutine(Subsystem subsystem, DoubleConsumer runVolts) {
    final String logName = subsystem.getName() + "/SysIdState";

    return new SysIdRoutine(
        new SysIdRoutine.Config(
            null, null, null, state -> Logger.recordOutput(logName, state.toString())),
        new SysIdRoutine.Mechanism(voltage -> runVolts.accept(voltage.in(Volts)), null, subsystem));
  }
}
