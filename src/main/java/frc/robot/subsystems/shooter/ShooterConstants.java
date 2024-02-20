package frc.robot.subsystems.shooter;

import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.TunableNumberWrapper;

public final class ShooterConstants {
  private static final TunableNumberWrapper tunable =
      new TunableNumberWrapper(ShooterConstants.class);
  public static final int TOP_MOTOR_ID = 14;
  public static final int BOTTOM_MOTOR_ID = 15;
  public static final LoggedTunableNumber RUN_VOLTS = tunable.makeField("run volts", 6);
  public static final LoggedTunableNumber AMP_VELOCITY_RAD_PER_SEC =
      tunable.makeField("amp rad per sec", 200);
  public static final LoggedTunableNumber SPEAKER_VELOCITY_RAD_PER_SEC =
      tunable.makeField("speaker rad per sec", 0.0);
  public static final double CLOSED_LOOP_RAMP_RATE = 0.01;
  public static final double OPEN_LOOP_RAMP_RATE = 0.01;
  public static final LoggedTunableNumber VELOCITY_TOLERANCE =
      tunable.makeField("velocity tolerance", 0.08);
  public static final LoggedTunableNumber AUTO_SPEAKER_SHOOT_VELOCITY =
      tunable.makeField("velocity tolerance", 1.0);

  public enum ShooterWheels {
    TOP,
    BOTTOM
  }

  public static final class Real {
    public static final class FeedForwardConstants {
      public static final class TopConstants {
        public static final LoggedTunableNumber kS = tunable.makeField("top/kS", 0.2);
        public static final LoggedTunableNumber kV = tunable.makeField("top/kV", 0.05);
      }

      public static final class BottomConstants {
        public static final LoggedTunableNumber kS = tunable.makeField("bottom/kS", 0.2);
        public static final LoggedTunableNumber kV = tunable.makeField("bottom/kV", 0.05);
      }
    }

    public static final class PIDConstants {
      public static final class TopConstants {
        public static final LoggedTunableNumber kP = tunable.makeField("top/kP", 0.1);
        public static final LoggedTunableNumber kI = tunable.makeField("top/kI", 0.0);
        public static final LoggedTunableNumber kD = tunable.makeField("top/kD", 0.0);
      }

      public static final class BottomConstants {
        public static final LoggedTunableNumber kP = tunable.makeField("bottom/kP", 0.1);
        public static final LoggedTunableNumber kI = tunable.makeField("bottom/kI", 0.0);
        public static final LoggedTunableNumber kD = tunable.makeField("bottom/kD", 0.0);
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
