package frc.robot.util.interpolation;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class InterpolationMaps {

  public static final InterpolatingDoubleTreeMap angleToHoldVolts =
      InterpolationMapBuilder.makeMap(InterpolationData.armHoldData);

  public static final InterpolatingDoubleTreeMap getShooterDistanceToArmAngle =
      InterpolationMapBuilder.makeMap(InterpolationData.shooterDistanceData, 0, 1);
  public static final InterpolatingDoubleTreeMap shooterDistanceToVelocity =
      InterpolationMapBuilder.makeMap(InterpolationData.shooterDistanceData, 0, 2);
}
