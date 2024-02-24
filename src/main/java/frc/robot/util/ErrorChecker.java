package frc.robot.util;

import frc.robot.subsystems.arm.ArmIOInputsAutoLogged;
import frc.robot.subsystems.climber.ClimberIOInputsAutoLogged;
import frc.robot.subsystems.drive.ModuleIOInputsAutoLogged;
import frc.robot.subsystems.intake.IntakeIOInputsAutoLogged;
import frc.robot.subsystems.shooter.ShooterIOInputsAutoLogged;

public class ErrorChecker {
  private ErrorChecker() {};

  public static void checkError(ShooterIOInputsAutoLogged inputs) {
    // Print error messages
    if (inputs.motorBrownOut) {
      System.err.println("MOTOR CAN ID " + inputs.motorCANID + " BROWNOUT");
    }
    if (inputs.motorSensorFault) {
      System.err.println("MOTOR CAN ID " + inputs.motorCANID + " SENSOR FAULT");
    }
  }

  public static void checkError(ModuleIOInputsAutoLogged inputs) {
    // Print error messages
    if (inputs.driveMotorBrownOut) {
      System.err.println("MOTOR CAN ID " + inputs.driveMotorCANID + " BROWNOUT");
    }
    if (inputs.driveMotorSensorFault) {
      System.err.println("MOTOR CAN ID " + inputs.driveMotorCANID + " SENSOR FAULT");
    }
    if (inputs.turnMotorBrownOut) {
      System.err.println("MOTOR CAN ID " + inputs.turnMotorCANID + " BROWNOUT");
    }
    if (inputs.turnMotorSensorFault) {
      System.err.println("MOTOR CAN ID " + inputs.turnMotorCANID + " SENSOR FAULT");
    }
  }

  public static void checkError(ArmIOInputsAutoLogged inputs) {
    // Print error messages
    if (inputs.leftMotorBrownOut) {
      System.err.println("MOTOR CAN ID " + inputs.leftMotorCANID + " BROWNOUT");
    }
    if (inputs.leftMotorSensorFault) {
      System.err.println("MOTOR CAN ID " + inputs.leftMotorCANID + " SENSOR FAULT");
    }
    if (inputs.rightMotorBrownOut) {
      System.err.println("MOTOR CAN ID " + inputs.rightMotorCANID + " BROWNOUT");
    }
    if (inputs.rightMotorSensorFault) {
      System.err.println("MOTOR CAN ID " + inputs.rightMotorCANID + " SENSOR FAULT");
    }
  }

  public static void checkError(IntakeIOInputsAutoLogged inputs) {
    // Print error messages
    if (inputs.motorBrownOut) {
      System.err.println("MOTOR CAN ID " + inputs.motorCANID + " BROWNOUT");
    }
    if (inputs.motorSensorFault) {
      System.err.println("MOTOR CAN ID " + inputs.motorCANID + " SENSOR FAULT");
    }
  }

  public static void checkError(ClimberIOInputsAutoLogged inputs) {
    // Print error messages
    if (inputs.motorBrownOut) {
      System.err.println("MOTOR CAN ID " + inputs.motorCANID + " BROWNOUT");
    }
    if (inputs.motorSensorFault) {
      System.err.println("MOTOR CAN ID " + inputs.motorCANID + " SENSOR FAULT");
    }
  }
}
