package frc.robot.subsystems.shooter;

import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.TunableNumberWrapper;
import java.lang.invoke.MethodHandles;

public final class ShooterConstants {
  private static final TunableNumberWrapper tunableTable =
      new TunableNumberWrapper(MethodHandles.lookup().lookupClass());
  public static final int TOP_MOTOR_ID = 14;
  public static final int BOTTOM_MOTOR_ID = 15;
  public static final LoggedTunableNumber RUN_VOLTS = tunableTable.makeField("run volts", 6);
  public static final LoggedTunableNumber IDLE_VOLTS = tunableTable.makeField("idle volts", 1);
  public static final LoggedTunableNumber AMP_VELOCITY_RAD_PER_SEC =
      tunableTable.makeField("amp rad per sec", 150);
  public static final LoggedTunableNumber SPEAKER_VELOCITY_RAD_PER_SEC =
      tunableTable.makeField("speaker rad per sec", 280);
  public static final LoggedTunableNumber PODIUM_VELOCITY_RAD_PER_SEC =
      tunableTable.makeField("podium rad per sec", 300);
  public static final double CLOSED_LOOP_RAMP_RATE = 0.01;
  public static final double OPEN_LOOP_RAMP_RATE = 0.01;
  public static final LoggedTunableNumber VELOCITY_TOLERANCE =
      tunableTable.makeField("velocity tolerance rad per s", 5);

  public static final LoggedTunableNumber AUTO_SHOOTER_TIMEOUT =
      tunableTable.makeField("auto shooter timeout", 2);

  public static final double TOP_GEAR_RATIO = 1;
  public static final double BOTTOM_GEAR_RATIO = 1;

  public enum ShooterWheels {
    TOP,
    BOTTOM
  }

  private record FlywheelConstants(double ks, double kv, double kp) {}

  private record ShooterTune(FlywheelConstants top, FlywheelConstants bottom) {}
  ;

  private static final ShooterTune mainTune =
      new ShooterTune(
          new FlywheelConstants(0.045715, 0.019471, 0.0001),
          new FlywheelConstants(0.18493, 0.02069, 0.0001));

  public static final class FlywheelModelConstants {
    public static final class Top {
      public static final LoggedTunableNumber kS =
          tunableTable.makeField("top/kS", mainTune.top.ks);
      public static final LoggedTunableNumber kV =
          tunableTable.makeField("top/kV", mainTune.top.kv);
      public static final LoggedTunableNumber kP =
          tunableTable.makeField("top/kP", mainTune.top.kp);
    }

    public static final class Bottom {
      public static final LoggedTunableNumber kS =
          tunableTable.makeField("bottom/kS", mainTune.bottom.ks);
      public static final LoggedTunableNumber kV =
          tunableTable.makeField("bottom/kV", mainTune.bottom.kv);
      public static final LoggedTunableNumber kP =
          tunableTable.makeField("bottom/kP", mainTune.bottom.kp);
    }
  }
}
