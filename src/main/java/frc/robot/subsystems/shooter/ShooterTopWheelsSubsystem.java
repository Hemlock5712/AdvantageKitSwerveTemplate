package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterTopWheelsSubsystem extends SubsystemBase {
  private final CANSparkMax motor;
  private final RelativeEncoder encoder;
  private final SimpleMotorFeedforward feedForward;
  private final PIDController pidController;

  public ShooterTopWheelsSubsystem() {
    motor =
        new CANSparkMax(
            Constants.ShooterConstants.TopWheelsConstants.MOTOR_ID,
            CANSparkLowLevel.MotorType.kBrushless);

    encoder = motor.getEncoder();

    configMotor();
    configEncoder();

    feedForward =
        new SimpleMotorFeedforward(
            Constants.ShooterConstants.TopWheelsConstants.FeedForwardConstants.kS,
            Constants.ShooterConstants.TopWheelsConstants.FeedForwardConstants.kV);

    pidController =
        new PIDController(
            Constants.ShooterConstants.TopWheelsConstants.PIDControllerConstants.kP,
            Constants.ShooterConstants.TopWheelsConstants.PIDControllerConstants.kI,
            Constants.ShooterConstants.TopWheelsConstants.PIDControllerConstants.kD);
  }

  public void setVelocity(double velocityRadPerSec) {
    setVoltage(calculateVoltage(velocityRadPerSec));
  }

  private void setVoltage(double voltage) {
    voltage = MathUtil.clamp(voltage, -11.9, 11.9);
    motor.setVoltage(voltage);
  }

  private double calculateVoltage(double velocityRadPerSec) {
    return feedForward.calculate(velocityRadPerSec)
        + pidController.calculate(this.getVelocity(), velocityRadPerSec);
  }

  public double getVelocity() {
    return encoder.getVelocity();
  }

  public double getTemperature() {
    return motor.getMotorTemperature();
  }

  public void stop() {
    motor.set(0);
  }

  private void configMotor() {
    motor.restoreFactoryDefaults();
    motor.setCANTimeout(250);
    motor.setSmartCurrentLimit(40);
    motor.enableVoltageCompensation(12.0);
    motor.burnFlash();
  }

  private void configEncoder() {
    encoder.setPosition(0);
    encoder.setPositionConversionFactor(Constants.ShooterConstants.ROT_TO_RAD);
    encoder.setVelocityConversionFactor(Constants.ShooterConstants.RAD_PER_SEC);
  }
}
