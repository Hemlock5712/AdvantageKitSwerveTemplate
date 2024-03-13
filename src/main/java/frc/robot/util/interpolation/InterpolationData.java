package frc.robot.util.interpolation;

import java.util.Arrays;

public class InterpolationData {

  protected static double[][] armHoldDataRaw = {
    /* rad position, upper hold volts, lower hold volts */
    {-1, 1.35, 0.4},
    {-0.1, 1.35, 0.4},
    {0.1777, 1.4, 0.4},
    {0.332, 1.1, 0.3},
    {0.482, 1, 0.4},
    {0.67, 0.8, 0.25},
    {0.74, 0.75, 0.25},
    {0.8, 0.7, 0.1},
    {1.04, 0.3, 0},
    {1.5, 0, -0.4},
    {3, -2, -2},
  };

  protected static double[][] armHoldData =
      Arrays.stream(armHoldDataRaw)
          .map(
              row -> {
                double upper = row[1];
                double lower = row[2];

                double middle = (upper + lower) / 2;
                double kS = (upper - lower) / 2;

                return new double[] {row[0], middle, kS};
              })
          .toArray(double[][]::new);

  protected static final double[][] shooterDistanceData = {
    /* distance m, angle rad, velocity rad/s */
    {0, 0.2, 280},
    {1.143, 0.2, 280},
    {2.3622, 0.426759297, 280},
    {2.9718, 0.502168204, 300},
    {3.5814, 0.466016954, 250},
    {4.191, 0.625507763, 350},
    {4.192, 0, 0},
    {100, 0, 0}
  };
}
