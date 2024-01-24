package frc.robot.subsystems.shooter;

import com.revrobotics.*;
import edu.wpi.first.math.util.Units;

public class ShooterIOSparkMax implements ShooterIO {
  private final CANSparkMax leader =
      new CANSparkMax(ShooterConstants.TOP_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax follower =
      new CANSparkMax(ShooterConstants.BOTTOM_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
  private final RelativeEncoder encoder = leader.getEncoder();
  private final SparkPIDController pidController = leader.getPIDController();

  public ShooterIOSparkMax() {
    leader.restoreFactoryDefaults();
    follower.restoreFactoryDefaults();

    leader.setCANTimeout(250);
    follower.setCANTimeout(250);

    leader.setInverted(false);
    follower.follow(leader, false);

    leader.enableVoltageCompensation(12.0);
    leader.setSmartCurrentLimit(30);

    leader.setClosedLoopRampRate(ShooterConstants.CLOSED_LOOP_RAMP_RATE);
    leader.setOpenLoopRampRate(ShooterConstants.OPEN_LOOP_RAMP_RATE);

    leader.burnFlash();
    follower.burnFlash();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition());
    inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());
    inputs.appliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
    inputs.currentAmps = new double[] {leader.getOutputCurrent(), follower.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
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
    leader.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pidController.setP(kP, 0);
    pidController.setI(kI, 0);
    pidController.setD(kD, 0);
    pidController.setFF(0, 0);
  }
}
