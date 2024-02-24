package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.beamBreak.BeamBreak;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterStateHelpers {
    private final ShooterSubsystem shooter;
    private final ArmSubsystem arm;
    private final BeamBreak beamBreak;

    public ShooterStateHelpers(ShooterSubsystem shooter, ArmSubsystem arm, BeamBreak beamBreak) {
        this.shooter = shooter;
        this.arm = arm;
        this.beamBreak = beamBreak;
    }

    public boolean armReadyToShoot() {
        return arm.atSetpoint() && arm.isPositionControlActive() && arm.getSetpointRad() > 0.1;
    }

    public boolean shooterReady() {
        return shooter.bothShootersNearTargetVelocity() &&
                shooter.getTargetVelocityRadPerSec() > 0;
    }

    public boolean canShoot() {
        return armReadyToShoot() && shooterReady() && beamBreak.detectNote();
    }

    public boolean canShootWithoutNoteDetection() {
        return armReadyToShoot() && shooterReady();
    }

    public Command waitUntilCanShoot() {
        return Commands.waitUntil(this::canShoot);
    }
    public Command waitUntilCanShootWithoutNoteDetection() {
        return Commands.waitUntil(this::canShoot);
    }

    public Command waitUntilCanShootAuto() {
        return waitUntilCanShootWithoutNoteDetection()
                .withTimeout(ShooterConstants.AUTO_SHOOTER_TIMEOUT.get());
    }
}
