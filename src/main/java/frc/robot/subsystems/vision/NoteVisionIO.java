// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface NoteVisionIO {
  @AutoLog
  public static class NoteVisionIOInputs {
    public double[] notePitches = {};
    public double[] noteYaws = {};
  }

  default void updateInputs(NoteVisionIOInputs inputs) {}
}
