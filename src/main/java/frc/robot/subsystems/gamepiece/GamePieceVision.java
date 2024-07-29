package frc.robot.subsystems.gamepiece;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.gamepiece.GamePieceVisionIO.GamePieceVisionIOInputs;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class GamePieceVision extends SubsystemBase {
  private final GamePieceVisionIO[] io;
  private final GamePieceVisionIOInputs[] inputs;

  private static final String VISION_PATH = "GamePiece/Inst";

  private ArrayList<GamePiece> allGamepiece = new ArrayList<>();

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
    ArrayList<GamePiece> tempGamePieces = new ArrayList<>();
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs(VISION_PATH + Integer.toString(i), inputs[i]);
      for (GamePiece gamePiece : inputs[i].gamePieces) {
        tempGamePieces.add(gamePiece);
      }
    }
    setAllGamePieces(tempGamePieces);
  }

  public ArrayList<GamePiece> getGamePiece() {
    return allGamepiece;
  }

  private void setAllGamePieces(ArrayList<GamePiece> tempGamePieces) {
    allGamepiece = tempGamePieces;
  }

  public record GamePiece(Translation2d translationOfGamepiece) {}
}
