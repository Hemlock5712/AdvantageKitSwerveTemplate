package frc.robot.subsystems.intake;

import com.revrobotics.*;
import edu.wpi.first.math.util.Units;

public class IntakeIOSparkMax implements IntakeIO {
  private static final double GEAR_RATIO = IntakeConstants.GEAR_RATIO;

  private final CANSparkMax motor =
      new CANSparkMax(IntakeConstants.MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();
  private final SparkPIDController pid = motor.getPIDController();

  public IntakeIOSparkMax() {
    motor.restoreFactoryDefaults();
    motor.setCANTimeout(250);
    motor.enableVoltageCompensation(12.0);
    motor.setSmartCurrentLimit(30);
    motor.setClosedLoopRampRate(IntakeConstants.CLOSED_LOOP_RAMP_RATE);
    motor.setOpenLoopRampRate(IntakeConstants.OPEN_LOOP_RAMP_RATE);
    motor.burnFlash();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition() / GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = new double[] {motor.getOutputCurrent()};
    inputs.temperature = motor.getMotorTemperature();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double feedForwardVolts) {
    pid.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * GEAR_RATIO,
        CANSparkBase.ControlType.kVelocity,
        0,
        feedForwardVolts,
        SparkPIDController.ArbFFUnits.kVoltage);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setP(kP, 0);
    pid.setI(kI, 0);
    pid.setD(kD, 0);
    pid.setFF(0, 0);
  }
}
