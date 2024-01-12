package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private final ShooterTopWheelsSubsystem shooterTopWheelsSubsystem;
  private final ShooterBottomWheelsSubsystem shooterBottomWheelsSubsystem;

  public ShooterSubsystem() {
    shooterTopWheelsSubsystem = new ShooterTopWheelsSubsystem();
    shooterBottomWheelsSubsystem = new ShooterBottomWheelsSubsystem();
  }

  public double getVelocity() {
    return (getTopVelocity() + getBottomVelocity()) / 2.0;
  }

  public double getTopVelocity() {
    return shooterTopWheelsSubsystem.getVelocity();
  }

  public double getBottomVelocity() {
    return shooterBottomWheelsSubsystem.getVelocity();
  }

  public double getTopTemperature() {
    return shooterTopWheelsSubsystem.getTemperature();
  }

  public double getBottomTemperature() {
    return shooterBottomWheelsSubsystem.getTemperature();
  }

  public void setVelocity(double velocity) {
    shooterTopWheelsSubsystem.setVelocity(velocity);
    shooterBottomWheelsSubsystem.setVelocity(velocity);
  }

  public void stop() {
    shooterTopWheelsSubsystem.stop();
    shooterBottomWheelsSubsystem.stop();
  }
}
