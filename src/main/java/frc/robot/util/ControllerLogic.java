package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.intake.IntakeConstants;

public class ControllerLogic {
    private final CommandXboxController driverController;
    private final CommandXboxController secondController;
    public ControllerLogic(CommandXboxController driverController, CommandXboxController secondController) {
        this.driverController = driverController;
        this.secondController = secondController;
    }

    public double getIntakeSpeed() {
        return MathUtil.clamp(
                driverController.getLeftTriggerAxis()
                        - driverController.getRightTriggerAxis()
                        + secondController.getLeftTriggerAxis()
                        - secondController.getRightTriggerAxis(),
                -1,
                1
        );
    }

    public Trigger getExtakeTrigger() {
        return new Trigger(() ->
                getIntakeSpeed() < -IntakeConstants.INTAKE_SPEED_THRESHOLD.get());
    }
    public Trigger getIntakeTrigger() {
        return new Trigger(() ->
                getIntakeSpeed() > IntakeConstants.INTAKE_SPEED_THRESHOLD.get());
    }
}
