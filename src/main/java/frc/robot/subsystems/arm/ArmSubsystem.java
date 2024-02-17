package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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

  private LoggedDashboardNumber kP = new LoggedDashboardNumber("ArmSubsystem/kP", ArmConstants.kP);
  private LoggedDashboardNumber kI = new LoggedDashboardNumber("ArmSubsystem/kI", ArmConstants.kI);
  private LoggedDashboardNumber kD = new LoggedDashboardNumber("ArmSubsystem/kD", ArmConstants.kD);
  private LoggedDashboardNumber kS = new LoggedDashboardNumber("ArmSubsystem/kS", ArmConstants.kS);
  private LoggedDashboardNumber kG = new LoggedDashboardNumber("ArmSubsystem/kG", ArmConstants.kG);
  private LoggedDashboardNumber kV = new LoggedDashboardNumber("ArmSubsystem/kV", ArmConstants.kV);
  private LoggedDashboardNumber kA = new LoggedDashboardNumber("ArmSubsystem/kA", ArmConstants.kA);

  public final SysIdRoutine sysid;

  private final InterpolatingDoubleTreeMap angleToHoldVolts = new InterpolatingDoubleTreeMap();

  @AutoLogOutput
  private Pose3d getArmPose() {
    return new Pose3d(0, 0, 0, new Rotation3d(0, armIOInputs.positionRad, 0));
  }

  public ArmSubsystem(ArmIO armIO) {
    this.armIO = armIO;
    SmartDashboard.putData(this);
    double shooterAngle = 70;
    arm.append(new MechanismLigament2d("intake", .3, -shooterAngle));
    arm.append(new MechanismLigament2d("shooter", .2, 180 - shooterAngle));

    sysid =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                state -> Logger.recordOutput("ArmSubsystem/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                voltage -> {
                  setVoltage(voltage.in(Volts));
                },
                null,
                this));

    //    angleToHoldVolts.put(0.067593724, 1.204724371);
    angleToHoldVolts.put(-1., 1.);
    angleToHoldVolts.put(0.067897516, 0.897637814);
    angleToHoldVolts.put(0.2207049, 0.874015778);
    angleToHoldVolts.put(0.269767311, 0.66141732);
    angleToHoldVolts.put(0.2816152, 0.566929132);
    //    angleToHoldVolts.put(0.293918777, 0.425196871);
    angleToHoldVolts.put(0.3615125, 0.448818907);
    //    angleToHoldVolts.put(0.379891918, 0.448818907);
    angleToHoldVolts.put(0.487758347, 0.354330719);
    //    angleToHoldVolts.put(0.538927039, 0.472440943);
    angleToHoldVolts.put(0.618824339, 0.377952754);
    angleToHoldVolts.put(0.7323615, 0.25984253);
    angleToHoldVolts.put(0.800188802, 0.118110236);
    angleToHoldVolts.put(0.876895658, 0.188976377);
    angleToHoldVolts.put(0.944489381, 0.118110236);
    angleToHoldVolts.put(0.971653791, 0.118110236);
    angleToHoldVolts.put(1.15881465, 0.);
    angleToHoldVolts.put(1.26878736, 0.);
    angleToHoldVolts.put(1.3, 0.);
    angleToHoldVolts.put(Math.PI, -1.);
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
      //      if (setpoint < 0.05 && armIOInputs.positionRad < 0.05) {
      //        active = false;
      //      }

      double pidVolts = pidController.calculate(armIOInputs.positionRad);
      //      double ffVolts = feedforward.calculate(armIOInputs.positionRad,
      // Math.signum(pidVolts));
      double holdVolts = angleToHoldVolts.get(armIOInputs.positionRad);
      double frictionVolts = feedforward.ks * Math.signum(pidVolts);

      double volts = holdVolts + pidVolts + frictionVolts;

      volts =
          MathUtil.clamp(volts, -ArmConstants.MAX_ARM_PID_VOLTS, ArmConstants.MAX_ARM_PID_VOLTS);

      Logger.recordOutput("Arm/hold volts", holdVolts);
      Logger.recordOutput("Arm/pid volts", pidVolts);
      Logger.recordOutput("Arm/frictions volts", frictionVolts);

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
    Logger.recordOutput("ArmSubsystem/attemptedVolts", volts);
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
