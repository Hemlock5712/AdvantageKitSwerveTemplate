package frc.robot.subsystems.gamepiece;

import frc.robot.util.VisionHelpers.GamePiece;
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
            "Game Piece " + Integer.toString(gamePiecePosition) + "/width", gamePiece.width());
        table.put(
            "Game Piece " + Integer.toString(gamePiecePosition) + "/hight ", gamePiece.hight());
      }
      gamePieces.clear();
    }

    @Override
    public void fromLog(LogTable table) {
      int gamePieceCount = table.get("Game Piece", 0);
      for (int i = 0; i < gamePieceCount; i++) {
        double x = table.get("Game Piece " + Integer.toString(i) + "/width ", 0.0);
        double y = table.get("Game Piece " + Integer.toString(i) + "/hight ", 0.0);
        gamePieces.add(new GamePiece(x, y));
      }
      gamePieces.clear();
    }
  }

  default void updateInputs(GamePieceVisionIOInputs inputs) {}
}
