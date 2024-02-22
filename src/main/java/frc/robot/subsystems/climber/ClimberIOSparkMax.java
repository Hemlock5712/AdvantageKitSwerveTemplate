package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;

public class ClimberIOSparkMax implements ClimberIO {
  private final CANSparkMax motor;

  private final DigitalInput limitSwitch;
  private final RelativeEncoder encoder;

  public ClimberIOSparkMax(int sparkMaxCanID, int limitSwitchDIOPort) {
    motor = new CANSparkMax(sparkMaxCanID, CANSparkLowLevel.MotorType.kBrushless);
    motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    motor.setInverted(false);
    limitSwitch = new DigitalInput(limitSwitchDIOPort);
    encoder = motor.getEncoder();
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.atBottom = !limitSwitch.get();
    inputs.positionRotations = encoder.getPosition();
    inputs.motorTemperatureCelsius = motor.getMotorTemperature();
  }

  @Override
  public void resetEncoder() {
    encoder.setPosition(0);
  }

  @Override
  public void toggleMotorInversion() {
    motor.setInverted(!motor.getInverted());
  }

  @Override
  public void setVoltage(double volts) {
    volts = MathUtil.clamp(volts, -11, 11);
    motor.setVoltage(volts);
  }
}
