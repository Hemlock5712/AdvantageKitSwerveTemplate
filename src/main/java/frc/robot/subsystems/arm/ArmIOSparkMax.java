package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.ColorSensor.ColorSensorIO;
import frc.robot.subsystems.shooter.ShooterConstants;
import org.littletonrobotics.junction.Logger;

public class ArmIOSparkMax implements ArmIO {
  private final CANSparkMax leader =
          new CANSparkMax(ArmConstants.LEFT_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax follower =
          new CANSparkMax(ArmConstants.RIGHT_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
  private final Encoder encoder = new Encoder(ArmConstants.ENCODER_PORT_1, ArmConstants.ENCODER_PORT_2);
  public ArmIOSparkMax() {
    follower.follow(leader, false);
    //todo tune encoder
  }


  @Override
  public void updateInputs(ArmIOInputs inputs) {
    // todo update switches
    inputs.positionRad = encoder.getDistance();
    inputs.velocityRad = encoder.getRate();
    inputs.appliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
    inputs.currentAmps = new double[] {leader.getOutputCurrent(), follower.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double volts) {
    volts = MathUtil.clamp(volts, -11.9, 11.9);
    leader.setVoltage(volts);
  }
}
