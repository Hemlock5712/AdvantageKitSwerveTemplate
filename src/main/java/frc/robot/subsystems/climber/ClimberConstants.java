package frc.robot.subsystems.climber;

import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.TunableNumberWrapper;
import java.lang.invoke.MethodHandles;

public class ClimberConstants {
  private static final TunableNumberWrapper tunableTable =
      new TunableNumberWrapper(MethodHandles.lookup().lookupClass());

  public static final int LEFT_MOTOR_ID = 11;
  public static final int RIGHT_MOTOR_ID = 12;
  public static final LoggedTunableNumber CLIMBER_VOLTS = tunableTable.makeField("volts", 10);
  public static final LoggedTunableNumber CLIMBER_RESET_VOLTS = tunableTable.makeField("reset volts", 7);

  public static final class RotationPositions {
    public static final double INITIAL_FULL_EXTENSION = 65;
    public static final double HIGHEST_FULL_EXTENSION = 80;
    public static final double FULL_EXTENSION_WIGGLE_ROOM =
        HIGHEST_FULL_EXTENSION - INITIAL_FULL_EXTENSION;
    public static final double FULL_EXTENSION_MIDDLE =
        (INITIAL_FULL_EXTENSION + HIGHEST_FULL_EXTENSION) / 2.;
    public static final double HIGH_END_FOR_BOTTOM =
        INITIAL_FULL_EXTENSION + HIGHEST_FULL_EXTENSION;
  }

  public static final double CLIMBER_RANGE_METERS = Units.inchesToMeters(11);

  public static final int LEFT_LIMIT_SWITCH_DIO_PORT = 1;
  public static final int RIGHT_LIMIT_SWITCH_DIO_PORT = 2;
}
