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

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  class ClimberIOInputs {
    public boolean atBottom = false;
    public double positionRotations = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double motorTemperatureCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(ClimberIOInputs inputs) {}

  default void setVoltage(double volts) {}

  default void resetEncoder() {}

  default void toggleMotorInversion() {}

  default void stop() {
    setVoltage(0);
  }
}
