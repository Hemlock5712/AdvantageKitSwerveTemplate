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

package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  class ArmIOInputs {
    public boolean lowerLimit = false;
    public boolean upperLimit = false;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
    public double leftMotorTemperatureCelsius = 0.0;
    public double rightMotorTemperatureCelsius = 0.0;
    public boolean leftMotorSensorFault = false;
    public boolean leftMotorBrownOut = false;
    public int leftMotorCANID = -1;
    public boolean rightMotorSensorFault = false;
    public boolean rightMotorBrownOut = false;
    public int rightMotorCANID = -1;
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(ArmIOInputs inputs) {}

  default void setVoltage(double volts) {}

  default void stop() {
    setVoltage(0);
  }
}
