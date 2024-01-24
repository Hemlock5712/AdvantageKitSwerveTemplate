package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import edu.wpi.first.math.util.Units;

public class IntakeIOTalonSRX implements IntakeIO {
  private static final double GEAR_RATIO = IntakeConstants.GEAR_RATIO;
  private final TalonSRX motor = new TalonSRX(IntakeConstants.MOTOR_ID);

  public IntakeIOTalonSRX() {
    TalonSRXConfiguration config = new TalonSRXConfiguration();
    config.peakCurrentLimit = 40;
    config.peakCurrentDuration = 1500;
    config.continuousCurrentLimit = 30;
    config.openloopRamp = IntakeConstants.OPEN_LOOP_RAMP_RATE;
    config.closedloopRamp = IntakeConstants.CLOSED_LOOP_RAMP_RATE;
    motor.configAllSettings(config);
    motor.enableVoltageCompensation(true);
    motor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.positionRad =
        Units.rotationsToRadians(motor.getSensorCollection().getQuadraturePosition() / GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(
            motor.getSensorCollection().getQuadratureVelocity() / GEAR_RATIO);
    inputs.appliedVolts = motor.getMotorOutputVoltage();
    inputs.currentAmps = new double[] {motor.getStatorCurrent()};
  }

  @Override
  public void setVoltage(double volts) {
    motor.set(ControlMode.PercentOutput, volts / 12.0);
  }

  @Override
  public void stop() {
    motor.set(ControlMode.PercentOutput, 0);
  }
}
