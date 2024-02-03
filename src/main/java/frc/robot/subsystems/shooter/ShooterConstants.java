package frc.robot.subsystems.shooter;

public final class ShooterConstants {
  public static final int TOP_MOTOR_ID = 9;
  public static final int BOTTOM_MOTOR_ID = 10;
  public static final double AMP_VELOCITY_RAD_PER_SEC = 2 * Math.PI;
  public static final double SPEAKER_VELOCITY_RAD_PER_SEC = 0.0;
  public static final double CLOSED_LOOP_RAMP_RATE = 0.2;
  public static final double OPEN_LOOP_RAMP_RATE = 0.2;

  public enum ShooterWheels {
    TOP,
    BOTTOM
  }

  public static final class Real {
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