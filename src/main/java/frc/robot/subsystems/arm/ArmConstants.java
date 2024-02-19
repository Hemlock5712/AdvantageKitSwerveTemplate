package frc.robot.subsystems.arm;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

public class ArmConstants {
  public static final int LEFT_MOTOR_ID = 9;
  public static final int RIGHT_MOTOR_ID = 10;
  public static final double kS = .1; // todo tune
  public static final double kG = 1.5; // to-a do tune
  public static final double kV = 0; // todo tune
  public static final double kA = 0.0; // todo tune
  public static final double kP = 4; // todo tune
  public static final double kI = 0.0; // todo tune
  public static final double kD = 0; // todo tune
  public static final double ARM_ENCODER_OFFSET_RAD = 2.1338351287912601;
  public static final int DUTY_CYCLE_ENCODER_PORT = 0;
  public static final int UPPER_LIMIT_SWITCH_PORT = 3;
  public static final int LOWER_LIMIT_SWITCH_PORT = 4;

  public static final double MIN_RAD = 0; // todo tune
  public static final double MAX_RAD = Units.degreesToRadians(95); // todo tune
  public static final double MAX_ARM_PID_VOLTS = 4.0;
  public static final double MANUAL_ARM_MAX_VOLTS = 5.0;

  // public static final int UPPER_LIMIT_SWITCH_PORT = 5;
  // public static final int LOWER_LIMIT_SWITCH_PORT = 6;

  public static class Positions {
    public static final double INTAKE_POS_RAD = -.1; // todo tune
    public static final double SPEAKER_POS_RAD = 0.2; // todo tune
    public static final double AMP_POS_RAD = Units.degreesToRadians(90); // todo tune
  }

  public static final InterpolatingDoubleTreeMap angleToHoldVolts =
      new InterpolatingDoubleTreeMap();

  static {
    //    angleToHoldVolts.put(0.067593724, 1.204724371);
    angleToHoldVolts.put(-1., 1.);
    angleToHoldVolts.put(0.067897516, 0.897637814);
    angleToHoldVolts.put(0.2207049, 0.874015778);
    angleToHoldVolts.put(0.269767311, 0.66141732);
    angleToHoldVolts.put(0.2816152, 0.566929132);
    //    angleToHoldVolts.put(0.293918777, 0.425196871);
    angleToHoldVolts.put(0.3615125, 0.448818907);
    //    angleToHoldVolts.put(0.379891918, 0.448818907);
    angleToHoldVolts.put(0.487758347, 0.354330719);
    //    angleToHoldVolts.put(0.538927039, 0.472440943);
    angleToHoldVolts.put(0.618824339, 0.377952754);
    angleToHoldVolts.put(0.7323615, 0.25984253);
    angleToHoldVolts.put(0.800188802, 0.118110236);
    angleToHoldVolts.put(0.876895658, 0.188976377);
    angleToHoldVolts.put(0.944489381, 0.118110236);
    angleToHoldVolts.put(0.971653791, 0.118110236);
    angleToHoldVolts.put(1.15881465, 0.);
    angleToHoldVolts.put(1.26878736, 0.);
    angleToHoldVolts.put(1.3, 0.);
    angleToHoldVolts.put(Math.PI, -1.);
  }
}
