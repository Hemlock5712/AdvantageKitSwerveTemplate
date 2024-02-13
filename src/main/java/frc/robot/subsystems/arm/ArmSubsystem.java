package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class ArmSubsystem extends SubsystemBase {
  private final ArmIO armIO;
  private final ArmIOInputsAutoLogged armIOInputs = new ArmIOInputsAutoLogged();
  private ArmFeedforward feedforward =
      new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA);
  public final PIDController pidController =
      new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
  @AutoLogOutput private final Mechanism2d mech = new Mechanism2d(2, 2);
  private final MechanismRoot2d root = mech.getRoot("arm", .7, .30);
  private final MechanismLigament2d arm = root.append(new MechanismLigament2d("arm", .8, 0));

  private double setpoint = 0.0;
  private boolean active = false;

  private LoggedDashboardNumber kP = new LoggedDashboardNumber("arm/kP", ArmConstants.kP);
  private LoggedDashboardNumber kI = new LoggedDashboardNumber("arm/kI", ArmConstants.kI);
  private LoggedDashboardNumber kD = new LoggedDashboardNumber("arm/kD", ArmConstants.kD);
  private LoggedDashboardNumber kS = new LoggedDashboardNumber("arm/kS", ArmConstants.kS);
  private LoggedDashboardNumber kG = new LoggedDashboardNumber("arm/kG", ArmConstants.kG);
  private LoggedDashboardNumber kV = new LoggedDashboardNumber("arm/kV", ArmConstants.kV);
  private LoggedDashboardNumber kA = new LoggedDashboardNumber("arm/kA", ArmConstants.kA);

  public ArmSubsystem(ArmIO armIO) {
    this.armIO = armIO;
    SmartDashboard.putData(this);
    double shooterAngle = 70;
    arm.append(new MechanismLigament2d("intake", .3, -shooterAngle));
    arm.append(new MechanismLigament2d("shooter", .2, 180 - shooterAngle));
  }

  private void updateControlConstants() {
    if (kP.get() != pidController.getP()) {
      pidController.setP(kP.get());
    }
    if (kI.get() != pidController.getI()) {
      pidController.setI(kI.get());
    }
    if (kD.get() != pidController.getD()) {
      pidController.setD(kD.get());
    }
    if (kS.get() != feedforward.ks) {
      feedforward = new ArmFeedforward(kS.get(), feedforward.kg, feedforward.kv, feedforward.ka);
    }
    if (kG.get() != feedforward.kg) {
      feedforward = new ArmFeedforward(feedforward.ks, kG.get(), feedforward.kv, feedforward.ka);
    }
    if (kV.get() != feedforward.kv) {
      feedforward = new ArmFeedforward(feedforward.ks, feedforward.kg, kV.get(), feedforward.ka);
    }
    if (kA.get() != feedforward.ka) {
      feedforward = new ArmFeedforward(feedforward.ks, feedforward.kg, feedforward.kv, kA.get());
    }
  }

  @Override
  public void periodic() {
    armIO.updateInputs(armIOInputs);
    Logger.processInputs("arm", armIOInputs);
    arm.setAngle(Units.radiansToDegrees(armIOInputs.positionRad));

    if (active) {
      double pidVolts = pidController.calculate(armIOInputs.positionRad);
      double ffVolts = feedforward.calculate(armIOInputs.positionRad, Math.signum(pidVolts));

      double volts = ffVolts + pidVolts;

      volts =
          MathUtil.clamp(volts, -ArmConstants.MAX_ARM_PID_VOLTS, ArmConstants.MAX_ARM_PID_VOLTS);

      Logger.recordOutput("Arm/ff volts", ffVolts);
      Logger.recordOutput("Arm/pid volts", pidVolts);

      setVoltage(volts);
    }

    updateControlConstants();
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
}
