package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
  private final ClimberIO climberIO;
  private final ClimberIOInputsAutoLogged climberIOInputs = new ClimberIOInputsAutoLogged();
  private String descriptor;

  public ClimberSubsystem(ClimberIO climberIO, String descriptor) {
    this.climberIO = climberIO;
    this.descriptor = descriptor;

    climberIO.resetEncoder();
  }

  @Override
  public void periodic() {
    climberIO.updateInputs(climberIOInputs);
    Logger.processInputs("climber/" + descriptor, climberIOInputs);
  }

  public void stop() {
    climberIO.stop();
  }

  public void setVoltage(double volts) {
    climberIO.setVoltage(volts);
  }

  @AutoLogOutput(key = "Climber/{descriptor}/atBottom")
  public boolean atBottom() {
    return climberIOInputs.atBottom;
  }

  public void resetEncoder() {
    climberIO.resetEncoder();
  }

  public void toggleInvert() {
    climberIO.toggleMotorInversion();
  }

  @AutoLogOutput(key = "Climber/{descriptor}/pastFullExtension")
  public boolean pastFullExtension() {
    return climberIOInputs.positionRotations > ClimberConstants.FULL_EXTENSION_ROTATIONS;
  }

  @AutoLogOutput(key = "Climber/{descriptor}/getPositionMeters")
  public double getPositionMeters() {
    return climberIOInputs.positionRotations;
  }
}
