package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.TunableNumberWrapper;
import java.lang.invoke.MethodHandles;

public class ArmConstants {
  private static final TunableNumberWrapper tunableTable =
      new TunableNumberWrapper(MethodHandles.lookup().lookupClass());

  public static final int LEFT_MOTOR_ID = 9;
  public static final int RIGHT_MOTOR_ID = 10;
  public static final LoggedTunableNumber kP = tunableTable.makeField("kP", 3);
  public static final LoggedTunableNumber kI = tunableTable.makeField("kI", 0.0);
  public static final LoggedTunableNumber kD = tunableTable.makeField("kD", 0);
  public static final LoggedTunableNumber setpointToleranceRad =
      tunableTable.makeField("setpoint tolerance rad", 0.01); // todo tune
  public static final double ARM_ENCODER_OFFSET_RAD = 2.1338351287912601;
  public static final int DUTY_CYCLE_ENCODER_PORT = 0;
  public static final int UPPER_LIMIT_SWITCH_PORT = 3;
  //  public static final int LOWER_LIMIT_SWITCH_PORT = 4;

  public static final double MIN_RAD = 0; // todo tune
  public static final double MAX_RAD = Units.degreesToRadians(90); // todo tune
  public static final LoggedTunableNumber MAX_ARM_PID_VOLTS =
      tunableTable.makeField("max arm pid volts", 3.0);
  public static final LoggedTunableNumber MANUAL_ARM_MAX_VOLTS =
      tunableTable.makeField("max arm manual volts", 2.0);

  public static final double MAX_ARM_VOLTS = 4;
  public static final double MOTOR_TO_ARM_RATIO = 20.0 * 60 / 24;

  public static class Positions {
    public static final LoggedTunableNumber INTAKE_POS_RAD =
        tunableTable.makeField("Intake pos", 0);
    public static final LoggedTunableNumber SPEAKER_POS_RAD =
        tunableTable.makeField("speaker pos", 0.27); // todo tune
    public static final LoggedTunableNumber AMP_POS_RAD =
        tunableTable.makeField("amp pos", 1.53); // todo tune
    public static final LoggedTunableNumber UPPER_DRIVE_RAD =
        tunableTable.makeField("upper drive pos", 1.3);
    public static final LoggedTunableNumber LOWER_DRIVE_RAD =
        tunableTable.makeField("lower drive pos", 0.15);
    public static final LoggedTunableNumber SPEAKER_FROM_PODIUM_POS_RAD =
        tunableTable.makeField("podium shot", 0.642);
  }
}
