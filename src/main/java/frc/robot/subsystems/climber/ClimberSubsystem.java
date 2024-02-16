package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;

public class ClimberSubsystem extends SubsystemBase {
  private final ClmiberIO clmiberIO;
  private final ClimberIOInputsAutoLogged ClimberIOInputs = new ClimberIOInputsAutoLogged();

  public ClimberSubsystem(ClmiberIO clmiberIO) {
    this.clmiberIO = clmiberIO;

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

  public boolean atBottom() {
    return ClimberIOInputs.atBottom;
  }

  public void resetEncoder() {
    clmiberIO.resetEncoder();
  }

  public void toggleInvert() {
    clmiberIO.toggleMotorInversion();
  }

  @AutoLogOutput
  public boolean pastFullExtension() {
    return ClimberIOInputs.positionRotations > ClimberConstants.FULL_EXTENSION_ROTATIONS;
  }

  @AutoLogOutput
  public double getPositionMeters() {
    return ClimberIOInputs.positionRotations;
  }
}
