// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.ColorSensor;

import org.littletonrobotics.junction.AutoLog;

public interface ColorSensorIO {
  @AutoLog
  public static class ColorSensorIOInputs {
    public double red = 0;
    public double blue = 0;
    public double green = 0;
    public int IR = 0;
    public int proximity = 0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ColorSensorIOInputs inputs) {}
}
