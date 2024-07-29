package frc.robot.subsystems.gamepiece;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.gamepiece.GamePieceVision.GamePiece;
import java.util.ArrayList;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface GamePieceVisionIO {
  class GamePieceVisionIOInputs implements LoggableInputs {
    ArrayList<GamePiece> gamePieces = new ArrayList<>();

    @Override
    public void toLog(LogTable table) {
      table.put("Game Piece", gamePieces.size());
      for (GamePiece gamePiece : gamePieces) {
        int gamePiecePosition = gamePieces.indexOf(gamePiece);
        table.put(
            "Game Piece " + Integer.toString(gamePiecePosition) + "/translation",
            gamePiece.translationOfGamepiece());
      }
      gamePieces.clear();
    }

    @Override
    public void fromLog(LogTable table) {
      int gamePieceCount = table.get("Game Piece", 0);
      for (int i = 0; i < gamePieceCount; i++) {
        Translation2d translation =
            table.get("Game Piece " + Integer.toString(i) + "/translation", new Translation2d());
        gamePieces.add(new GamePiece(translation));
      }
      gamePieces.clear();
    }
  }

  default void updateInputs(GamePieceVisionIOInputs inputs) {}
}
