package frc.robot.subsystems.intake;

public final class IntakeConstants {
  public static final int MOTOR_ID = 13;
  public static final int MOTOR_ID_TALON = 23;
  public static final double MAX_RAD_PER_SEC = 0.0;
  public static final double AUTO_RAD_PER_SEC = 0.0;
  public static final double CLOSED_LOOP_RAMP_RATE = 0.2;
  public static final double OPEN_LOOP_RAMP_RATE = 0.2;
  public static final double GEAR_RATIO = 1.0;
  public static final double INTAKE_VOLTAGE = 5;

  public static final class Real {
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
