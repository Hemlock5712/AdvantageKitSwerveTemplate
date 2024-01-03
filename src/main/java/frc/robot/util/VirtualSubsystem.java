// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.ArrayList;
import java.util.List;

/** Represents a subsystem unit that requires a periodic callback but not a hardware mutex. */
public abstract class VirtualSubsystem {
  private static final List<VirtualSubsystem> subsystems = new ArrayList<>();

  protected VirtualSubsystem() {
    subsystems.add(this);
  }

  /** Calls {@link #periodic()} on all virtual subsystems. */
  public static void periodicAll() {
    subsystems.forEach(VirtualSubsystem::periodic);
  }

  /** This method is called periodically once per loop cycle. */
  public abstract void periodic();
}
