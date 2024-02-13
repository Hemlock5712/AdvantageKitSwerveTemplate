package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
  private static final double LOOP_PERIOD_SECS = 0.02;

  private final DCMotor gearbox = DCMotor.getNEO(2);
  private final SingleJointedArmSim arm =
      new SingleJointedArmSim(
          gearbox, 20.0 * 60 / 24, SingleJointedArmSim.estimateMOI(.7, 12.5), .7, 0, 1, true, 0);

  private double volts = 0.0;

  public ArmIOSim() {}

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    arm.update(LOOP_PERIOD_SECS);

    inputs.lowerLimit = arm.hasHitLowerLimit();
    inputs.upperLimit = arm.hasHitUpperLimit();
    inputs.positionRad = arm.getAngleRads();
    inputs.appliedVolts = volts;
    inputs.currentAmps = new double[] {arm.getCurrentDrawAmps() / 2, arm.getCurrentDrawAmps() / 2};
    inputs.leftMotorTemperatureCelsius = 25.0;
    inputs.rightMotorTemperatureCelsius = 25.0;

  }

  @Override
  public void setVoltage(double volts) {
    volts = MathUtil.clamp(volts, -11.9, 11.9);
    this.volts = volts;
    arm.setInputVoltage(volts);
  }
}
