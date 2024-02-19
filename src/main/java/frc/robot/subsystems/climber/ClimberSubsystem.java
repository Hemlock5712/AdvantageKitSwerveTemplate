package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberConstants.RotationPositions;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
  private final ClimberIO climberIO;
  private final ClimberIOInputsAutoLogged climberIOInputs = new ClimberIOInputsAutoLogged();
  private String descriptor;
  @AutoLogOutput private double volts = 0;

  public ClimberSubsystem(ClimberIO climberIO, String descriptor) {
    this.climberIO = climberIO;
    this.descriptor = descriptor;

    climberIO.resetEncoder();
  }

  @Override
  public void periodic() {
    climberIO.updateInputs(climberIOInputs);
    Logger.processInputs("climber/" + descriptor, climberIOInputs);
    checkAndStopIfAtBottom();
  }

  public void stop() {
    climberIO.stop();
  }

  private void checkAndStopIfAtBottom() {
    if (cannotUseVolts(volts)) {
      climberIO.setVoltage(0);
    }
  }

  @AutoLogOutput
  private boolean cannotUseVolts(double volts) {
    boolean pastFullExtension = pastFullExtension();
    boolean positiveVoltageDirection = volts > 0;

    return (climberIOInputs.atBottom
        && ((pastFullExtension && positiveVoltageDirection)
            || (!pastFullExtension && !positiveVoltageDirection)));
  }

  public void setVoltage(double volts) {
    if (!cannotUseVolts(volts)) {
      climberIO.setVoltage(volts);
      this.volts = volts;
    }
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
    return climberIOInputs.positionRotations > RotationPositions.FULL_EXTENSION_MIDDLE;
  }

  @AutoLogOutput(key = "Climber/{descriptor}/getPositionMeters")
  public double getPositionMeters() {
    if (climberIOInputs.positionRotations > RotationPositions.INITIAL_FULL_EXTENSION &&
        climberIOInputs.positionRotations < RotationPositions.HIGHEST_FULL_EXTENSION) {
      return ClimberConstants.CLIMBER_RANGE_METERS;
    }

    if (!pastFullExtension()) {
      return climberIOInputs.positionRotations
          / RotationPositions.INITIAL_FULL_EXTENSION
          * ClimberConstants.CLIMBER_RANGE_METERS;
    } else {
      return (RotationPositions.HIGH_END_FOR_BOTTOM - climberIOInputs.positionRotations)
          / RotationPositions.INITIAL_FULL_EXTENSION
          * ClimberConstants.CLIMBER_RANGE_METERS;
    }
  }
}
