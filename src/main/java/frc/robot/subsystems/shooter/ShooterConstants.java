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
  public static final LoggedTunableNumber AMP_VELOCITY_RAD_PER_SEC =
      tunableTable.makeField("amp rad per sec", 200);
  public static final LoggedTunableNumber SPEAKER_VELOCITY_RAD_PER_SEC =
      tunableTable.makeField("speaker rad per sec", 280);
  public static final double CLOSED_LOOP_RAMP_RATE = 0.01;
  public static final double OPEN_LOOP_RAMP_RATE = 0.01;
  public static final LoggedTunableNumber VELOCITY_TOLERANCE =
      tunableTable.makeField("velocity tolerance rad per s", 5);

  public static final LoggedTunableNumber AUTO_SHOOTER_TIMEOUT =
          tunableTable.makeField("auto shooter timeout", 3);

  public enum ShooterWheels {
    TOP,
    BOTTOM
  }

  public static final class Real {
    public static final class FeedForwardConstants {
      public static final class TopConstants {
        public static final LoggedTunableNumber kS = tunableTable.makeField("top/kS", 0.097527);
        public static final LoggedTunableNumber kV = tunableTable.makeField("top/kV", 0.02056);
      }

      public static final class BottomConstants {
        public static final LoggedTunableNumber kS = tunableTable.makeField("bottom/kS", 0.18271);
        public static final LoggedTunableNumber kV = tunableTable.makeField("bottom/kV", 0.021523);
      }
    }

    public static final class PIDConstants {
      public static final class TopConstants {
        public static final LoggedTunableNumber kP = tunableTable.makeField("top/kP", 0.00011219);
        public static final LoggedTunableNumber kI = tunableTable.makeField("top/kI", 0.0);
        public static final LoggedTunableNumber kD = tunableTable.makeField("top/kD", 0.0);
      }

      public static final class BottomConstants {
        public static final LoggedTunableNumber kP = tunableTable.makeField("bottom/kP", 0.0005);
        public static final LoggedTunableNumber kI = tunableTable.makeField("bottom/kI", 0.0);
        public static final LoggedTunableNumber kD = tunableTable.makeField("bottom/kD", 0.0);
      }
    }
  }

  public static final class Sim {
    public static final class FeedForwardConstants {
      public static final class TopConstants {
        public static final double kS = 0.2;
        public static final double kV = 0.05;
      }

      public static final class BottomConstants {
        public static final double kS = 0.2;
        public static final double kV = 0.05;
      }
    }

    public static final class PIDConstants {
      public static final class TopConstants {
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
      }

      public static final class BottomConstants {
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
      }
    }
  }
}
