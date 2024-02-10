package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends ProfiledPIDSubsystem {
  private final ArmIO armIO;
  private final ArmIOInputsAutoLogged armIOInputs = new ArmIOInputsAutoLogged();
  private final ArmFeedforward feedforward =
      new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA);
  private final PIDController pidController =
      new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
  @AutoLogOutput private final Mechanism2d mech = new Mechanism2d(2, 2);
  private final MechanismRoot2d root = mech.getRoot("arm", .7, .30);
  private final MechanismLigament2d arm = root.append(new MechanismLigament2d("arm", .8, 0));
  private double setpoint = 0.0;
  private boolean active = false;

  @AutoLogOutput
  private double getkP() {
    return pidController.getP();
  }

  public ArmSubsystem(ArmIO armIO) {
    super(
        new ProfiledPIDController(
            ArmConstants.kP,
            0,
            0,
            new TrapezoidProfile.Constraints(
                ArmConstants.MAX_VELOCITY_RAD_PER_SEC,
                ArmConstants.MAX_ACCELERATION_RAD_PER_SEC_SQUARED)),
        0);
    this.armIO = armIO;
    SmartDashboard.putData(this);
    double shooterAngle = 70;
    setGoal(ArmConstants.ARM_ENCODER_OFFSET_RAD);
    arm.append(new MechanismLigament2d("intake", .3, -shooterAngle));
    arm.append(new MechanismLigament2d("shooter", .2, 180 - shooterAngle));
  }

  @Override
  public void periodic() {
    armIO.updateInputs(armIOInputs);
    Logger.processInputs("arm", armIOInputs);
  }

  public void setPositionRad(double target) {
    active = true;
    setpoint = target;
    pidController.setSetpoint(target);
  }

  public void stop() {
    active = false;
    armIO.stop();
  }

  public void setVoltage(double volts) {
    if ((volts < 0 && armIOInputs.lowerLimit) || (volts > 0 && armIOInputs.upperLimit)) {
      volts = 0;
    }
    armIO.setVoltage(volts);
  }

  @AutoLogOutput
  public double getPositionRad() {
    return armIOInputs.positionRad;
  }

  @AutoLogOutput
  public double getOffset() {
    return pidController.getPositionError();
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    double feedforwardVolts = feedforward.calculate(setpoint.position, setpoint.velocity);
    armIO.setVoltage(output + feedforwardVolts);
  }

  @Override
  public double getMeasurement() {
    return armIO.getEncoderRadians();
  }
}
