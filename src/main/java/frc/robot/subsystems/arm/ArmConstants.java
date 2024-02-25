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
  public static final LoggedTunableNumber kS = tunableTable.makeField("kS", .1); // todo tune
  public static final LoggedTunableNumber kP = tunableTable.makeField("kP", 4); // todo tune
  public static final LoggedTunableNumber kI = tunableTable.makeField("kI", 0.0); // todo tune
  public static final LoggedTunableNumber kD = tunableTable.makeField("kD", 0); // todo tune
  public static final LoggedTunableNumber setpointToleranceRad =
      tunableTable.makeField("setpoint tolerance rad", 0.04); // todo tune
  public static final double ARM_ENCODER_OFFSET_RAD = 2.1338351287912601;
  public static final int DUTY_CYCLE_ENCODER_PORT = 0;
  public static final int UPPER_LIMIT_SWITCH_PORT = 3;
  public static final int LOWER_LIMIT_SWITCH_PORT = 4;

  public static final double MIN_RAD = 0; // todo tune
  public static final double MAX_RAD = Units.degreesToRadians(95); // todo tune
  public static final LoggedTunableNumber MAX_ARM_PID_VOLTS =
      tunableTable.makeField("max arm pid volts", 4.0);
  public static final LoggedTunableNumber MANUAL_ARM_MAX_VOLTS =
      tunableTable.makeField("max arm manual volts", 2.0);

  // public static final int UPPER_LIMIT_SWITCH_PORT = 5;
  // public static final int LOWER_LIMIT_SWITCH_PORT = 6;

  public static class Positions {
    public static final LoggedTunableNumber INTAKE_POS_RAD =
        tunableTable.makeField("Intake pos", 0);
    public static final LoggedTunableNumber SPEAKER_POS_RAD =
        tunableTable.makeField("speaker pos", 0.2); // todo tune
    public static final LoggedTunableNumber AMP_POS_RAD =
        tunableTable.makeField("amp pos", Units.degreesToRadians(90)); // todo tune
  }
}
