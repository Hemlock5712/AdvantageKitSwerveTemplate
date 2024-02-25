package frc.robot.subsystems.climber;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberConstants.RotationPositions;
import frc.robot.util.ErrorChecker;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
  private final ClimberIO climberIO;
  private final ClimberIOInputsAutoLogged climberIOInputs = new ClimberIOInputsAutoLogged();
  private String descriptor;
  @Getter private boolean beenReset = false;
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
    ErrorChecker.checkError(climberIOInputs);
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

  public void resetClimbersAssumingPositiveVoltageIsDown() {
    climberIO.resetEncoder();
    climberIO.toggleMotorInversion();
    beenReset = true;
  }

  @AutoLogOutput(key = "Climber/{descriptor}/pastFullExtension")
  public boolean pastFullExtension() {
    return climberIOInputs.positionRotations > RotationPositions.FULL_EXTENSION_MIDDLE;
  }

  @AutoLogOutput(key = "Climber/{descriptor}/positionMeters")
  public double getPositionMeters() {
    if (climberIOInputs.positionRotations > RotationPositions.INITIAL_FULL_EXTENSION
        && climberIOInputs.positionRotations < RotationPositions.HIGHEST_FULL_EXTENSION) {
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

  @AutoLogOutput(key = "Climber/{descriptor}/climberStagePoses")
  public Pose3d[] getClimberStagePositionPoses() {
    double pos = getPositionMeters();
    return new Pose3d[] {
      new Pose3d(0, 0, pos / 2, new Rotation3d()), new Pose3d(0, 0, pos, new Rotation3d())
    };
  }
}
