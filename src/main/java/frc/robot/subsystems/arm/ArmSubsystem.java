package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {
  private final ArmIO armIO;
  private final ArmIOInputsAutoLogged armIOInputs = new ArmIOInputsAutoLogged();
  private final ArmFeedforward feedforward =
          new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA);
  private final PIDController pidController = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
  private double setpoint = 0.0;
  private boolean active = false;

  public ArmSubsystem(ArmIO armIO) {
    this.armIO = armIO;
  }

  @Override
  public void periodic() {
    armIO.updateInputs(armIOInputs);
    Logger.processInputs("arm", armIOInputs);

    if (active) {
      double ffVolts = feedforward.calculate(setpoint, 0);
      double pidVolts = pidController.calculate(armIOInputs.positionRad);

      double volts = ffVolts + pidVolts;


      volts = MathUtil.clamp(volts, -12, 12);

      Logger.recordOutput("Arm/ff volts", ffVolts);
      Logger.recordOutput("Arm/pid volts", pidVolts);

      armIO.setVoltage(volts);
    }
  }

  public void setPositionRad(double target) {
    active = true;
    setpoint = target;
    pidController.setSetpoint(target);
  }

  public void stop() {
    active = false;
    armIO.stop();
  }

  public double getPositionRad() {
    return armIOInputs.positionRad;
  }
}
