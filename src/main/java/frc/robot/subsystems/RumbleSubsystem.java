package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RumbleSubsystem extends SubsystemBase {
  private final XboxController[] controllers;

  public RumbleSubsystem(XboxController... controllers) {
    this.controllers = controllers;
  }

  public void set(double strength) {
    set(GenericHID.RumbleType.kBothRumble, strength);
  }

  public void set(GenericHID.RumbleType rumbleType, double strength) {
    for (var controller : controllers) {
      controller.setRumble(rumbleType, strength);
    }
  }

  public void stop() {
    set(0);
  }

  public Command rumbleForTime(double seconds, double strength) {
    return startEnd(() -> set(strength), this::stop).withTimeout(seconds);
  }
}
