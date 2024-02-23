package frc.robot.util.interpolation;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class InterpolationMapBuilder {
  /**
   * @param data a 2d array where the first column is the input and the second is the output
   * @return a {@link InterpolatingDoubleTreeMap} that is populated with the data
   */
  public static InterpolatingDoubleTreeMap makeMap(double[][] data) {
    return makeMap(data);
  }

  /**
   * @param data      a 2d array where the rows are different measurements with input and output
   *                  columns
   * @param inputCol  the index of the column that has the input values/keys
   * @param outputCol the index of the column that has the output values
   * @return a {@link InterpolatingDoubleTreeMap} that is populated with the data given
   */
  public static InterpolatingDoubleTreeMap makeMap(double[][] data, int inputCol, int outputCol) {
    final InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();

    for (double[] row : data) {
      map.put(row[inputCol], row[outputCol]);
    }

    return map;
  }
}