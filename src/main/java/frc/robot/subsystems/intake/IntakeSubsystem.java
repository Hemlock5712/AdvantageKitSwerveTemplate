package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final SimpleMotorFeedforward feedForward;

  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
    feedForward = new SimpleMotorFeedforward(0.0, 0.0, 0.0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public void setVelocity(double velocityRadPerSec) {
    io.setVelocity(velocityRadPerSec, feedForward.calculate(velocityRadPerSec));
  }

  public void stop() {
    io.stop();
  }

  public double getVelocityRadPerSec() {
    return inputs.velocityRadPerSec;
  }
}
