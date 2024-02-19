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

package frc.robot.subsystems.beamBreak;

import org.littletonrobotics.junction.AutoLog;

public interface BeamBreakIO {
  @AutoLog
  class BeamBreakIOInputs {
    public boolean triggered = false;
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(BeamBreakIOInputs inputs) {}
}
