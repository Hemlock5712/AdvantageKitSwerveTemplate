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

package frc.robot;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.*;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.climber.ManualClimberCommand;
import frc.robot.commands.climber.ResetClimbers;
import frc.robot.subsystems.ColorSensor.ColorSensor;
import frc.robot.subsystems.ColorSensor.ColorSensorIO;
import frc.robot.subsystems.ColorSensor.ColorSensorIOReal;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSparkMax;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveController;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.AprilTagVisionIO;
import frc.robot.subsystems.vision.AprilTagVisionIOLimelight;
import frc.robot.subsystems.vision.AprilTagVisionIOPhotonVisionSIM;
import frc.robot.util.ShootingBasedOnPoseCalculator;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final AprilTagVision aprilTagVision;
  private static DriveController driveMode = new DriveController();
  private final ShooterSubsystem shooter;

  private final Intake intake;
  private final ColorSensor colorSensor;
  private final ArmSubsystem arm;
  private final ClimberSubsystem leftClimber;
  private final ClimberSubsystem rightClimber;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController secondController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  //   private final LoggedDashboardNumber flywheelSpeedInput =
  //       new LoggedDashboardNumber("Flywheel Speed", 1500.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.getMode()) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            /*         new ModuleIOTalonFX(moduleConfigs[0]),
            new ModuleIOTalonFX(moduleConfigs[1]),
            new ModuleIOTalonFX(moduleConfigs[2]),
            new ModuleIOTalonFX(moduleConfigs[3]));
             */
            new Drive(
                new GyroIONavX2(),
                new ModuleIOSparkMax(moduleConfigs[0]),
                new ModuleIOSparkMax(moduleConfigs[1]),
                new ModuleIOSparkMax(moduleConfigs[2]),
                new ModuleIOSparkMax(moduleConfigs[3]));

        aprilTagVision =
            new AprilTagVision(
                new AprilTagVisionIOLimelight("limelight"),
                new AprilTagVisionIOLimelight("limelight-two"));

        colorSensor = new ColorSensor(new ColorSensorIOReal());

        shooter =
            new ShooterSubsystem(
                new ShooterIOSparkMax(ShooterConstants.ShooterWheels.TOP),
                new ShooterIOSparkMax(ShooterConstants.ShooterWheels.BOTTOM));

        intake = new Intake(new IntakeIOSparkMax());

        arm = new ArmSubsystem(new ArmIOSparkMax());

        leftClimber =
            new ClimberSubsystem(
                new ClimberIOSparkMax(
                    ClimberConstants.LEFT_MOTOR_ID, ClimberConstants.LEFT_LIMIT_SWITCH_DIO_PORT),
                "left");
        rightClimber =
            new ClimberSubsystem(
                new ClimberIOSparkMax(
                    ClimberConstants.RIGHT_MOTOR_ID, ClimberConstants.RIGHT_LIMIT_SWITCH_DIO_PORT),
                "right");
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        // flywheel = new Flywheel(new FlywheelIOSim());
        aprilTagVision =
            new AprilTagVision(
                new AprilTagVisionIOPhotonVisionSIM(
                    "photonCamera1",
                    new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0)),
                    drive::getDrive));
        // flywheel = new Flywheel(new FlywheelIOSim());
        shooter = new ShooterSubsystem(new ShooterIO() {}, new ShooterIO() {});
        intake = new Intake(new IntakeIO() {});
        colorSensor = new ColorSensor(new ColorSensorIO() {});
        arm = new ArmSubsystem(new ArmIOSim());
        leftClimber = new ClimberSubsystem(new ClimberIO() {}, "left");
        rightClimber = new ClimberSubsystem(new ClimberIO() {}, "right");

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        // flywheel = new Flywheel(new FlywheelIO() {});
        aprilTagVision = new AprilTagVision(new AprilTagVisionIO() {});
        shooter = new ShooterSubsystem(new ShooterIO() {}, new ShooterIO() {});
        intake = new Intake(new IntakeIO() {});
        colorSensor = new ColorSensor(new ColorSensorIO() {});
        arm = new ArmSubsystem(new ArmIO() {});
        leftClimber = new ClimberSubsystem(new ClimberIO() {}, "left");
        rightClimber = new ClimberSubsystem(new ClimberIO() {}, "right");

        break;
    }

    // Set up auto routines
    // Arm
    NamedCommands.registerCommand(
        "Arm to ground intake position",
        ArmCommands.autoArmToPosition(arm, () -> ArmConstants.Positions.INTAKE_POS_RAD));
    NamedCommands.registerCommand(
        "Arm to amp position",
        ArmCommands.autoArmToPosition(arm, () -> ArmConstants.Positions.AMP_POS_RAD));
    NamedCommands.registerCommand(
        "Arm to speaker position",
        ArmCommands.autoArmToPosition(arm, () -> ArmConstants.Positions.SPEAKER_POS_RAD));
    NamedCommands.registerCommand(
        "Arm to calculated speaker angle",
        ArmCommands.autoArmToPosition(
            arm,
            () ->
                ShootingBasedOnPoseCalculator.calculateAngleInRadiansWithConstantVelocity(
                    drive.getPose())));

    // Intake
    NamedCommands.registerCommand(
        "Intake until note", new IntakeUntilNoteCommand(colorSensor, intake));

    // Shooter
    NamedCommands.registerCommand(
        "Shoot speaker",
        ShooterCommands.fullshot(
            shooter, intake, colorSensor, ShooterConstants.AUTO_SPEAKER_SHOOT_VELOCITY));

    AutoBuilder.buildAuto("MiddleTwoNote");

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    autoChooser.addOption(
        "Intake sysid quasistatic forward",
        intake.sysid.quasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Intake sysid quasistatic reverse",
        intake.sysid.quasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Intake sysid dynamic forward", intake.sysid.dynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Intake sysid dynamic reverse", intake.sysid.dynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Shooter sysid quasistatic forward",
        shooter.sysid.quasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Shooter sysid quasistatic reverse",
        shooter.sysid.quasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Shooter sysid dynamic forward", shooter.sysid.dynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Shooter sysid dynamic reverse", shooter.sysid.dynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Arm sysid quasistatic forward", arm.sysid.quasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Arm sysid quasistatic reverse", arm.sysid.quasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Arm sysid dynamic forward", arm.sysid.dynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Arm sysid dynamic reverse", arm.sysid.dynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    aprilTagVision.setDataInterfaces(drive::addVisionData);
    driveMode.setPoseSupplier(drive::getPose);
    driveMode.disableHeadingControl();
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private final LoggedDashboardNumber intakePos =
      new LoggedDashboardNumber("ArmSubsystem/intake rad", ArmConstants.Positions.INTAKE_POS_RAD);

  private final LoggedDashboardNumber speakerPos =
      new LoggedDashboardNumber("ArmSubsystem/speaker rad", ArmConstants.Positions.SPEAKER_POS_RAD);
  private final LoggedDashboardNumber ampPos =
      new LoggedDashboardNumber("ArmSubsystem/amp rad", ArmConstants.Positions.AMP_POS_RAD);

  private Command runShooterVolts;

  private void configureButtonBindings() {
    runShooterVolts =
        Commands.startEnd(
            () -> {
              shooter.runVolts(ShooterConstants.RUN_VOLTS);
            },
            shooter::stop,
            shooter);

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            driveMode,
            () -> -driverController.getRightY(),
            () -> -driverController.getRightX(),
            () -> driverController.getLeftX()));

    intake.setDefaultCommand(
        Commands.runEnd(
            () -> {
              intake.setVoltage(
                  IntakeConstants.INTAKE_VOLTAGE
                      * (driverController.getLeftTriggerAxis()
                          - driverController.getRightTriggerAxis()));
            },
            intake::stop,
            intake));

    driverController.rightBumper().whileTrue(runShooterVolts);

    driverController.a().onTrue(Commands.runOnce(drive::resetGyro));

    leftClimber.setDefaultCommand(
        new ManualClimberCommand(leftClimber, () -> -secondController.getLeftY()));
    rightClimber.setDefaultCommand(
        new ManualClimberCommand(rightClimber, () -> -secondController.getRightY()));

    secondController.leftBumper().whileTrue(runShooterVolts);
    secondController.rightBumper().whileTrue(new IntakeUntilNoteCommand(colorSensor, intake));

    secondController
        .a()
        .whileTrue(
            ArmCommands.manualArmCommand(
                arm,
                () ->
                    2
                        * (secondController.getLeftTriggerAxis()
                            - secondController.getRightTriggerAxis())));

    secondController.x().whileTrue(new ResetClimbers(leftClimber));
    secondController.b().whileTrue(new ResetClimbers(rightClimber));

    for (var controller : new CommandXboxController[] {driverController, secondController}) {
      configureUniversalControls(controller);
    }
  }

  private void configureUniversalControls(CommandXboxController controller) {
    controller.povDown().onTrue(ArmCommands.autoArmToPosition(arm, intakePos::get));
    controller.povRight().onTrue(ArmCommands.autoArmToPosition(arm, speakerPos::get));
    controller.povUp().onTrue(ArmCommands.autoArmToPosition(arm, ampPos::get));

    controller.povLeft().toggleOnTrue(runShooterVolts);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
