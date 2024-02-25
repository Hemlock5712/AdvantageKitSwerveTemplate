package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ArmIOSparkMax implements ArmIO {
  private final CANSparkMax leader =
      new CANSparkMax(ArmConstants.LEFT_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax follower =
      new CANSparkMax(ArmConstants.RIGHT_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
  private final DutyCycleEncoder encoder =
      new DutyCycleEncoder(ArmConstants.DUTY_CYCLE_ENCODER_PORT);

  private final DigitalInput upperLimitSwitch =
      new DigitalInput(ArmConstants.UPPER_LIMIT_SWITCH_PORT);
  private final DigitalInput lowerLimitSwitch =
      new DigitalInput(ArmConstants.LOWER_LIMIT_SWITCH_PORT);

  public ArmIOSparkMax() {
    // The motors are mirrored, so invert
    leader.setInverted(true);
    follower.follow(leader, true);
    encoder.reset();
    encoder.setDistancePerRotation(2 * Math.PI);
    follower.setIdleMode(CANSparkBase.IdleMode.kBrake);
    leader.setIdleMode(CANSparkBase.IdleMode.kBrake);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.positionRad =
        (encoder.getAbsolutePosition() * Math.PI * 2) - ArmConstants.ARM_ENCODER_OFFSET_RAD;
    inputs.upperLimit = (inputs.positionRad > ArmConstants.MAX_RAD);
    inputs.lowerLimit = (inputs.positionRad < ArmConstants.MIN_RAD);
    inputs.appliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
    inputs.currentAmps = new double[] {leader.getOutputCurrent(), follower.getOutputCurrent()};
    inputs.leftMotorTemperatureCelsius = leader.getMotorTemperature();
    inputs.rightMotorTemperatureCelsius = follower.getMotorTemperature();
    inputs.leftMotorSensorFault = leader.getFault(CANSparkBase.FaultID.kSensorFault);
    inputs.leftMotorBrownOut = leader.getFault(CANSparkBase.FaultID.kBrownout);
    inputs.leftMotorCANID = leader.getDeviceId();
    inputs.rightMotorSensorFault = follower.getFault(CANSparkBase.FaultID.kSensorFault);
    inputs.rightMotorBrownOut = follower.getFault(CANSparkBase.FaultID.kBrownout);
    inputs.rightMotorCANID = follower.getDeviceId();
  }

  @Override
  public void setVoltage(double volts) {
    volts = MathUtil.clamp(volts, -11.9, 11.9);
    leader.setVoltage(volts);
  }
}
