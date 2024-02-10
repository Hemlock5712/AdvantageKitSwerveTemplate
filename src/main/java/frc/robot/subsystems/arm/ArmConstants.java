package frc.robot.subsystems.arm;

public class ArmConstants {
  public static final int LEFT_MOTOR_ID = 9;
  public static final int RIGHT_MOTOR_ID = 10;
  public static final double kS = .2; // todo tune
  public static final double kG = 1.0; // todo tune
  public static final double kV = 0; // todo tune
  public static final double kA = 0.0; // todo tune
  public static final double kP = 2; // todo tune
  public static final double kI = 0.0; // todo tune
  public static final double kD = 0; // todo tune
  public static final double ARM_ENCODER_OFFSET_RAD = 0.8089522745415447;
  public static final int DUTY_CYCLE_ENCODER_PORT = 0;
  public static final int UPPER_LIMIT_SWITCH_PORT = 3; // todo fix
  public static final int LOWER_LIMIT_SWITCH_PORT = 4; // todo fix

  public static final double MIN_RAD = 0; // todo tune
  public static final double MAX_RAD = Math.PI / 3.; // todo tune
  public static final double MAX_ARM_PID_VOLTS = 7.0;
  public static final double MANUAL_ARM_MAX_VOLTS = 5.0;

  // public static final int UPPER_LIMIT_SWITCH_PORT = 5;
  // public static final int LOWER_LIMIT_SWITCH_PORT = 6;
}
