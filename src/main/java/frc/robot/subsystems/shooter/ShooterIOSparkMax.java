package frc.robot.subsystems.shooter;

import com.revrobotics.*;
import edu.wpi.first.math.util.Units;

public class ShooterIOSparkMax implements ShooterIO {
  private final CANSparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkPIDController pidController;

  public ShooterIOSparkMax(ShooterConstants.ShooterWheels topOrBottom) {
    switch (topOrBottom) {
      case TOP:
        motor =
            new CANSparkMax(ShooterConstants.TOP_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
        encoder = motor.getEncoder();
        encoder.setPositionConversionFactor(ShooterConstants.TOP_GEAR_RATIO);
        encoder.setVelocityConversionFactor(ShooterConstants.TOP_GEAR_RATIO);
        break;
      case BOTTOM:
        motor =
            new CANSparkMax(
                ShooterConstants.BOTTOM_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
        encoder = motor.getEncoder();
        encoder.setPositionConversionFactor(ShooterConstants.BOTTOM_GEAR_RATIO);
        encoder.setVelocityConversionFactor(ShooterConstants.BOTTOM_GEAR_RATIO);
        break;
      default:
        System.out.println("Shooter top/bottom not valid");
        motor = null;
        encoder = null;
        break;
    }
    motor.restoreFactoryDefaults();
    motor.setCANTimeout(250);
    motor.setInverted(false);
    motor.enableVoltageCompensation(12.0);
    motor.setSmartCurrentLimit(30);
    motor.setClosedLoopRampRate(ShooterConstants.CLOSED_LOOP_RAMP_RATE);
    motor.setOpenLoopRampRate(ShooterConstants.OPEN_LOOP_RAMP_RATE);
    motor.burnFlash();

    pidController = motor.getPIDController();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition());
    inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = new double[] {motor.getOutputCurrent()};
    inputs.motorTemperatureCelsius = motor.getMotorTemperature();
    inputs.motorSensorFault = motor.getFault(CANSparkBase.FaultID.kSensorFault);
    inputs.motorBrownOut = motor.getFault(CANSparkBase.FaultID.kBrownout);
    inputs.motorCANID = motor.getDeviceId();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double feedForwardVolts) {
    pidController.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec),
        CANSparkBase.ControlType.kVelocity,
        0,
        feedForwardVolts,
        SparkPIDController.ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pidController.setP(kP, 0);
    pidController.setI(kI, 0);
    pidController.setD(kD, 0);
    pidController.setFF(0, 0);
  }
}
