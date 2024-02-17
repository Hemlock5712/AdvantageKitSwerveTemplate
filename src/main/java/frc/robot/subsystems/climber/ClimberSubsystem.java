package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;

public class ClimberSubsystem extends SubsystemBase {
  private final ClmiberIO clmiberIO;
  private final ClimberIOInputsAutoLogged ClimberIOInputs = new ClimberIOInputsAutoLogged();
  private String descriptor;

  public ClimberSubsystem(ClmiberIO clmiberIO, String descriptor) {
    this.clmiberIO = clmiberIO;
    this.descriptor = descriptor;

    clmiberIO.resetEncoder();
  }

  @Override
  public void periodic() {
    clmiberIO.updateInputs(ClimberIOInputs);
  }

  public void stop() {
    clmiberIO.stop();
  }

  public void setVoltage(double volts) {
    clmiberIO.setVoltage(volts);
  }

  @AutoLogOutput(key = "Climber/{descriptor}/atBottom")
  public boolean atBottom() {
    return ClimberIOInputs.atBottom;
  }

  public void resetEncoder() {
    clmiberIO.resetEncoder();
  }

  public void toggleInvert() {
    clmiberIO.toggleMotorInversion();
  }

  @AutoLogOutput(key = "Climber/{descriptor}/pastFullExtension")
  public boolean pastFullExtension() {
    return ClimberIOInputs.positionRotations > ClimberConstants.FULL_EXTENSION_ROTATIONS;
  }

  @AutoLogOutput(key = "Climber/{descriptor}/getPositionMeters")
  public double getPositionMeters() {
    return ClimberIOInputs.positionRotations;
  }
}
