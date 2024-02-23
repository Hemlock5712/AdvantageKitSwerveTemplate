package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.interpolation.InterpolationMaps.angleToHoldVolts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {
  private final ArmIO armIO;
  private final ArmIOInputsAutoLogged armIOInputs = new ArmIOInputsAutoLogged();
  public final PIDController pidController =
      new PIDController(ArmConstants.kP.get(), ArmConstants.kI.get(), ArmConstants.kD.get());
  @AutoLogOutput private final Mechanism2d mech = new Mechanism2d(2, 2);
  private final MechanismRoot2d root = mech.getRoot("arm", .7, .30);
  private final MechanismLigament2d arm = root.append(new MechanismLigament2d("arm", .8, 0));

  private double setpoint = 0.0;
  private boolean active = false;

  public final SysIdRoutine sysid;

  private final Translation3d ROTATION_POINT = new Translation3d(-.26, 0.0, .273);

  @AutoLogOutput
  private Pose3d getArmPose() {
    Rotation3d rotation = new Rotation3d(0, -armIOInputs.positionRad, 0);
    Translation3d transformedPoint = ROTATION_POINT.minus(ROTATION_POINT.rotateBy(rotation));
    return new Pose3d(transformedPoint, rotation);
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
                  limitAndSetVolts(voltage.in(Volts));
                },
                null,
                this));
  }

  private void updateControlConstants() {
    pidController.setP(ArmConstants.kP.get());
    pidController.setI(ArmConstants.kI.get());
    pidController.setD(ArmConstants.kD.get());
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
      double frictionVolts = ArmConstants.kS.get() * Math.signum(pidVolts);

      double volts = pidVolts + frictionVolts;

      volts =
          MathUtil.clamp(
              volts, -ArmConstants.MAX_ARM_PID_VOLTS.get(), ArmConstants.MAX_ARM_PID_VOLTS.get());

      volts += holdVolts;

      Logger.recordOutput("Arm/hold volts", holdVolts);
      Logger.recordOutput("Arm/pid volts", pidVolts);
      Logger.recordOutput("Arm/friction volts", frictionVolts);

      limitAndSetVolts(volts);
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

  private void limitAndSetVolts(double volts) {
    if ((volts < 0 && armIOInputs.lowerLimit) || (volts > 0 && armIOInputs.upperLimit)) {
      volts = 0;
    }
    Logger.recordOutput("ArmSubsystem/attemptedVolts", volts);
    armIO.setVoltage(volts);
  }

  public void setManualVoltage(double volts) {
    active = false;
    armIO.setVoltage(volts);
    //    limitAndSetVolts(volts);
  }

  public boolean atTop() {
    return armIOInputs.upperLimit;
  }

  public boolean atBottom() {
    return armIOInputs.lowerLimit;
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
