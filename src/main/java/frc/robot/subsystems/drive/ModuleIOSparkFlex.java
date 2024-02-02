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

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.DriveConstants.ModuleConfig;

import java.util.Queue;

/**
 * Module IO implementation for SparkFlex drive motor controller, SparkFlex turn motor controller (NEO
 * or NEO 550), and analog absolute encoder connected to the RIO
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOSparkFlex implements ModuleIO {
  private final CANSparkFlex driveSparkFlex;
  private final CANSparkFlex turnSparkFlex;
  private final CANcoder cancoder;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnRelativeEncoder;
  private final StatusSignal<Double> turnAbsolutePosition;


  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  private final Rotation2d absoluteEncoderOffset;

  public ModuleIOSparkFlex(ModuleConfig config) {
    // Init motor & encoder objects
    driveSparkFlex = new CANSparkFlex(config.driveID(), MotorType.kBrushless);
    turnSparkFlex = new CANSparkFlex(config.turnID(), MotorType.kBrushless);
    cancoder = new CANcoder(config.absoluteEncoderChannel(), canbus);
    absoluteEncoderOffset = config.absoluteEncoderOffset(); // MUST BE CALIBRATED

    driveSparkFlex.restoreFactoryDefaults();
    turnSparkFlex.restoreFactoryDefaults();

    driveSparkFlex.setCANTimeout(250);
    turnSparkFlex.setCANTimeout(250);

    driveEncoder = driveSparkFlex.getEncoder();
    turnRelativeEncoder = turnSparkFlex.getEncoder();

    turnSparkFlex.setInverted(config.turnMotorInverted());
    driveSparkFlex.setSmartCurrentLimit(40);
    turnSparkFlex.setSmartCurrentLimit(30);
    driveSparkFlex.enableVoltageCompensation(12.0);
    turnSparkFlex.enableVoltageCompensation(12.0);

    driveEncoder.setPosition(0.0);
    driveEncoder.setMeasurementPeriod(10);
    driveEncoder.setAverageDepth(2);

    turnRelativeEncoder.setPosition(0.0);
    turnRelativeEncoder.setMeasurementPeriod(10);
    turnRelativeEncoder.setAverageDepth(2);

    driveSparkFlex.setCANTimeout(0);
    turnSparkFlex.setCANTimeout(0);

    driveSparkFlex.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / odometryFrequency));
    turnSparkFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus2, (int) (1000.0 / odometryFrequency));

    cancoder.getConfigurator().apply(new CANcoderConfiguration());

    timestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();

    
    drivePositionQueue =
        SparkMaxOdometryThread.getInstance().registerSignal(driveEncoder::getPosition);
    turnPositionQueue =
        SparkMaxOdometryThread.getInstance().registerSignal(turnRelativeEncoder::getPosition);

    driveSparkFlex.burnFlash();
    turnSparkFlex.burnFlash();

    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnAbsolutePosition.setUpdateFrequency(50.0);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad =
        Units.rotationsToRadians(driveEncoder.getPosition()) / moduleConstants.driveReduction();
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity())
            / moduleConstants.driveReduction();
    inputs.driveAppliedVolts = driveSparkFlex.getAppliedOutput() * driveSparkFlex.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {driveSparkFlex.getOutputCurrent()};

    turnAbsolutePosition.refresh();
    inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
            .minus(absoluteEncoderOffset);
    inputs.turnPosition =
        Rotation2d.fromRotations(
            turnRelativeEncoder.getPosition() / moduleConstants.turnReduction());
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
            / moduleConstants.turnReduction();
    inputs.turnAppliedVolts = turnSparkFlex.getAppliedOutput() * turnSparkFlex.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turnSparkFlex.getOutputCurrent()};

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble(
                (Double value) ->
                    Units.rotationsToRadians(value) / moduleConstants.driveReduction())
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map(
                (Double value) -> Rotation2d.fromRotations(value / moduleConstants.turnReduction()))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveSparkFlex.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkFlex.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveSparkFlex.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnSparkFlex.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
