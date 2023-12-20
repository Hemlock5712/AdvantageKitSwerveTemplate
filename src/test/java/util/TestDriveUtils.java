package frc.test.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.DriveUtils;
import org.junit.jupiter.api.Test;

public class TestDriveUtils {
  @Test
  public void testGetDistanceFromVectorReturnsZeroWhenNull() {
    assertEquals(0.0, DriveUtils.getDistanceFromVector(null, null));
    assertEquals(0.0, DriveUtils.getDistanceFromVector(new Translation2d(2, 2), null));
    assertEquals(0.0, DriveUtils.getDistanceFromVector(null, new Translation2d(2, 2)));
  }

  @Test
  public void testGetDistanceFromVectorReturnsCorrectValues() {
    assertEquals(
        0, DriveUtils.getDistanceFromVector(new Translation2d(2, 2), new Translation2d(1, 1)), 0.1);
    assertEquals(
        -Math.sqrt(2) / 2.0,
        DriveUtils.getDistanceFromVector(new Translation2d(2, 2), new Translation2d(1, 0)),
        0.1);
  }
}
