package frc.robot.subsystems;


import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax motor;
  private final RelativeEncoder encoder;
  private final SimpleMotorFeedforward feedForward;
  private final PIDController pidController;
  public IntakeSubsystem() {
    motor = new CANSparkMax(
            Constants.IntakeConstants.MOTOR_ID,
            CANSparkLowLevel.MotorType.kBrushless);

    encoder = motor.getEncoder();

    configMotor();
    configEncoder();

    feedForward = new SimpleMotorFeedforward(
            Constants.IntakeConstants.FeedForwardConstants.kS,
            Constants.IntakeConstants.FeedForwardConstants.kV);

    pidController = new PIDController(
            Constants.IntakeConstants.PIDControllerConstants.kP,
            Constants.IntakeConstants.PIDControllerConstants.kI,
            Constants.IntakeConstants.PIDControllerConstants.kD);

  }

  public void setVelocity(double velocityRadPerSec) {
    setVoltage(calculateVoltage(velocityRadPerSec));
  }

  private void setVoltage(double voltage) {
    voltage = MathUtil.clamp(voltage, -11.9, 11.9);
    motor.setVoltage(voltage);
  }

  private double calculateVoltage(double velocityRadPerSec) {
    return feedForward.calculate(velocityRadPerSec) + pidController.calculate(this.getVelocity(), velocityRadPerSec);
  }

  public double getVelocity() {
    return encoder.getVelocity();
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
    encoder.setPositionConversionFactor(Constants.IntakeConstants.ROT_TO_RAD);
    encoder.setVelocityConversionFactor(Constants.IntakeConstants.RAD_PER_SEC);
  }
}

