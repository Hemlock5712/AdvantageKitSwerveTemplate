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

import static frc.robot.subsystems.drive.DriveConstants.moduleConfigs;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.ArmCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.MultiDistanceShot;
import frc.robot.commands.climber.ManualClimberCommand;
import frc.robot.commands.climber.ResetClimberBasic;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.beamBreak.BeamBreak;
import frc.robot.subsystems.beamBreak.BeamBreakIO;
import frc.robot.subsystems.beamBreak.BeamBreakIOReal;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSparkMax;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.drive.DriveController.DriveModeType;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.AprilTagVisionIO;
import frc.robot.subsystems.vision.AprilTagVisionIOLimelight;
import frc.robot.subsystems.vision.AprilTagVisionIOPhotonVisionSIM;
import frc.robot.util.FieldConstants;
import frc.robot.util.ShooterStateHelpers;
import frc.robot.util.ShootingBasedOnPoseCalculator;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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
  private static final DriveController driveMode = new DriveController();
  private final ShooterSubsystem shooter;

  private final Intake intake;
  private final ArmSubsystem arm;
  private final ClimberSubsystem leftClimber;
  private final ClimberSubsystem rightClimber;
  private final BeamBreak beamBreak;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController secondController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final Command resetClimbersCommand;
  private final ShooterStateHelpers shooterStateHelpers;

  //   private final LoggedTunableNumber flywheelSpeedInput =
  //       new LoggedTunableNumber("Flywheel Speed", 1500.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.getMode()) {
      case REAL -> {
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
        beamBreak = new BeamBreak(new BeamBreakIOReal());
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
      }
      case SIM -> {
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
        arm = new ArmSubsystem(new ArmIOSim());
        leftClimber = new ClimberSubsystem(new ClimberIO() {}, "left");
        rightClimber = new ClimberSubsystem(new ClimberIO() {}, "right");
        beamBreak = new BeamBreak(new BeamBreakIO() {});
      }
      default -> {
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
        arm = new ArmSubsystem(new ArmIO() {});
        leftClimber = new ClimberSubsystem(new ClimberIO() {}, "left");
        rightClimber = new ClimberSubsystem(new ClimberIO() {}, "right");
        beamBreak = new BeamBreak(new BeamBreakIO() {});
      }
    }

    shooterStateHelpers = new ShooterStateHelpers(shooter, arm, beamBreak);

    configureNamedCommands();

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    configureAutoChooser();

    resetClimbersCommand =
        ResetClimberBasic.on(leftClimber).alongWith(ResetClimberBasic.on(rightClimber));

    // Configure the button bindings
    aprilTagVision.setDataInterfaces(drive::addVisionData);
    driveMode.setPoseSupplier(drive::getPose);
    driveMode.disableHeadingControl();
    configureButtonBindings();
  }

  private void configureNamedCommands() {
    // Set up auto routines
    // Arm
    NamedCommands.registerCommand(
        "Arm to ground intake position",
        ArmCommands.autoArmToPosition(arm, ArmConstants.Positions.INTAKE_POS_RAD::get));
    NamedCommands.registerCommand(
        "Arm to amp position",
        ArmCommands.autoArmToPosition(arm, ArmConstants.Positions.AMP_POS_RAD::get));
    NamedCommands.registerCommand(
        "Arm to speaker position",
        ArmCommands.autoArmToPosition(arm, ArmConstants.Positions.SPEAKER_POS_RAD::get));
    NamedCommands.registerCommand(
        "Arm to calculated speaker angle",
        Commands.runOnce(
            () ->
                Logger.recordOutput(
                    "arm/targetShootingAngle",
                    ShootingBasedOnPoseCalculator.calculateAngleInRadiansWithConstantVelocity(
                        drive.getPose()))));
    //        ArmCommands.autoArmToPosition(
    //            arm,
    //            () ->
    //                ShootingBasedOnPoseCalculator.calculateAngleInRadiansWithConstantVelocity(
    //                    drive.getPose())));

    // Intake
    NamedCommands.registerCommand(
        "Intake until note", IntakeCommands.untilNote(intake, beamBreak::detectNote));

    // Shooter
    NamedCommands.registerCommand(
        "shoot speaker",
        ArmCommands.autoArmToPosition(arm, ArmConstants.Positions.SPEAKER_POS_RAD::get)
            .andThen(
                Commands.runOnce(() -> shooter.runVolts(ShooterConstants.RUN_VOLTS.get()), shooter))
            .andThen(shooterStateHelpers.waitUntilCanShootAuto())
            .andThen(
                Commands.runOnce(
                    () -> intake.setVoltage(IntakeConstants.INTAKE_VOLTAGE.get()), intake))
            .andThen(Commands.waitUntil(() -> !beamBreak.detectNote()).withTimeout(1))
            .andThen(Commands.runOnce(() -> shooter.runVolts(0), shooter))
            .andThen(Commands.runOnce(() -> intake.setVoltage(0), intake))
            .andThen(
                ArmCommands.autoArmToPosition(arm, ArmConstants.Positions.INTAKE_POS_RAD::get)));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private Command runShooterVelocity;

  private void configureButtonBindings() {
    runShooterVelocity =
        Commands.startEnd(
            () -> shooter.runVelocity(ShooterConstants.SPEAKER_VELOCITY_RAD_PER_SEC.get()),
            shooter::stop,
            shooter);

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            driveMode,
            () -> -driverController.getRightY(),
            () -> -driverController.getRightX(),
            () -> -driverController.getLeftX()));

    driverController
        .leftBumper()
        .whileTrue(
            Commands.startEnd(
                () -> shooter.runVolts(ShooterConstants.RUN_VOLTS.get()), shooter::stop, shooter));

    driveMode.setDriveMode(DriveModeType.SPEAKER);
    driverController
        .y()
        .toggleOnTrue(
            Commands.startEnd(driveMode::enableHeadingControl, driveMode::disableHeadingControl));

    intake.setDefaultCommand(
        IntakeCommands.manualIntakeCommand(
            intake,
            () ->
                driverController.getLeftTriggerAxis()
                    - driverController.getRightTriggerAxis()
                    + secondController.getLeftTriggerAxis()
                    - secondController.getRightTriggerAxis(),
            () -> !beamBreak.detectNote() || shooterStateHelpers.canShoot()));

    driverController.a().onTrue(Commands.runOnce(drive::resetGyro));

    driverController
        .povRight()
        .whileTrue(
            new MultiDistanceShot(
                drive::getPose, FieldConstants.Speaker.centerSpeakerOpening, shooter, arm));

    leftClimber.setDefaultCommand(
        new ManualClimberCommand(leftClimber, () -> -secondController.getLeftY()));
    rightClimber.setDefaultCommand(
        new ManualClimberCommand(rightClimber, () -> -secondController.getRightY()));

    secondController
        .leftBumper()
        .whileTrue(IntakeCommands.untilNote(intake, beamBreak::detectNote));

    //    secondController
    //        .a()
    //        .whileTrue(
    //            ArmCommands.manualArmCommand(
    //                arm,
    //                () ->
    //                    2
    //                        * (secondController.getLeftTriggerAxis()
    //                            - secondController.getRightTriggerAxis())));

    secondController.x().onTrue(ResetClimberBasic.on(leftClimber));
    secondController.b().onTrue(ResetClimberBasic.on(rightClimber));

    for (var controller : new CommandXboxController[] {driverController, secondController}) {
      configureUniversalControls(controller);
    }

    //    final CommandXboxController debugController = new CommandXboxController(2);
    //
    //    arm.setDefaultCommand(
    //        Commands.run(
    //            () ->
    //                arm.setManualVoltage(
    //                    2
    //                        * (debugController.getRightTriggerAxis()
    //                            - debugController.getLeftTriggerAxis())),
    //            arm));
  }

  private void configureUniversalControls(CommandXboxController controller) {
    controller
        .povDown()
        .onTrue(ArmCommands.autoArmToPosition(arm, ArmConstants.Positions.INTAKE_POS_RAD::get));
    controller
        .povLeft()
        .onTrue(ArmCommands.autoArmToPosition(arm, ArmConstants.Positions.SPEAKER_POS_RAD::get));
    controller
        .povUp()
        .onTrue(ArmCommands.autoArmToPosition(arm, ArmConstants.Positions.AMP_POS_RAD::get));

    controller.rightBumper().whileTrue(runShooterVelocity);
  }

  private void configureAutoChooser() {
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
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return resetClimbersCommand.asProxy().alongWith(autoChooser.get().asProxy());
  }

  public Command getTeleopCommand() {
    return resetClimbersCommand;
  }
}
