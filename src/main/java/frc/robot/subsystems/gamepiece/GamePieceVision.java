package frc.robot.subsystems.gamepiece;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.gamepiece.GamePieceVisionIO.GamePieceVisionIOInputs;
import org.littletonrobotics.junction.Logger;

public class GamePieceVision extends SubsystemBase {
  private final GamePieceVisionIO[] io;
  private final GamePieceVisionIOInputs[] inputs;

  private static final String VISION_PATH = "GamePiece/Inst";

  public GamePieceVision(GamePieceVisionIO... io) {
    System.out.println("[Init] Creating GamePieceVision");
    this.io = io;
    inputs = new GamePieceVisionIOInputs[io.length];
    for (int i = 0; i < io.length; i++) {
      inputs[i] = new GamePieceVisionIOInputs();
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs(VISION_PATH + Integer.toString(i), inputs[i]);
    }
  }
}
