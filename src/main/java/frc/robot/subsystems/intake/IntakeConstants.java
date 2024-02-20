package frc.robot.subsystems.intake;

import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.TunableNumberWrapper;
import java.lang.invoke.MethodHandles;

public final class IntakeConstants {
  private static final TunableNumberWrapper tunableTable =
      new TunableNumberWrapper(MethodHandles.lookup().lookupClass());
  public static final int MOTOR_ID = 13;
  public static final int MOTOR_ID_TALON = 23;
  public static final double CLOSED_LOOP_RAMP_RATE = 0.2;
  public static final double OPEN_LOOP_RAMP_RATE = 0.2;
  public static final double GEAR_RATIO = 1.0;
  public static final LoggedTunableNumber INTAKE_VOLTAGE = tunableTable.makeField("intake volts", 6);

  public static final class Real {
    public static final class FeedForwardConstants {
      public static final LoggedTunableNumber kS = tunableTable.makeField("ks", 0.0);
      public static final LoggedTunableNumber kV = tunableTable.makeField("kv", 0.0);
    }

    public static final class PIDConstants {
      public static final LoggedTunableNumber kP = tunableTable.makeField("kp", 0.0);
      public static final LoggedTunableNumber kI = tunableTable.makeField("ki", 0.0);
      public static final LoggedTunableNumber kD = tunableTable.makeField("kd", 0.0);
    }
  }

  public static final class Sim {
    public static final class FeedForwardConstants {
      public static final double kS = 0.0;
      public static final double kV = 0.0;
    }

    public static final class PIDConstants {
      public static final double kP = 0.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
    }
  }
}
