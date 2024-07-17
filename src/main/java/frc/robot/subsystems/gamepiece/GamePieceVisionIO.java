package frc.robot.subsystems.gamepiece;

import frc.robot.util.VisionHelpers.GamePiece;
import java.util.ArrayList;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.targeting.TargetCorner;

public interface GamePieceVisionIO {
  class GamePieceVisionIOInputs implements LoggableInputs {
    ArrayList<GamePiece> gamePieces = new ArrayList<>();

    @Override
    public void toLog(LogTable table) {
      table.put("Game Piece", gamePieces.size());
      for (GamePiece gamePiece : gamePieces) {
        int gamePiecePosition = gamePieces.indexOf(gamePiece);
        table.put(
            "Game Piece " + Integer.toString(gamePiecePosition) + "/Corner",
            gamePiece.gamePiece().size());
        for (TargetCorner corners : gamePiece.gamePiece()) {
          int cornerPosition = gamePiece.gamePiece().indexOf(corners);
          table.put(
              "Game Piece "
                  + Integer.toString(gamePiecePosition)
                  + "/Corner "
                  + Integer.toString(cornerPosition)
                  + "/x",
              corners.x);
          table.put(
              "Game Piece "
                  + Integer.toString(gamePiecePosition)
                  + "/Corner "
                  + Integer.toString(cornerPosition)
                  + "/y",
              corners.y);
        }
      }
      gamePieces.clear();
    }

    @Override
    public void fromLog(LogTable table) {
      int gamePieceCount = table.get("Game Piece", 0);
      for (int i = 0; i < gamePieceCount; i++) {
        ArrayList<TargetCorner> corners = new ArrayList<>();
        int cornerCount = table.get("Game Piece " + Integer.toString(i) + "/Corner", 0);
        for (int j = 0; j < cornerCount; j++) {
          double x =
              table.get(
                  "Game Piece " + Integer.toString(i) + "/Corner " + Integer.toString(j) + "/x",
                  0.0);
          double y =
              table.get(
                  "Game Piece " + Integer.toString(i) + "/Corner " + Integer.toString(j) + "/y",
                  0.0);
          corners.add(new TargetCorner(x, y));
        }
        gamePieces.add(new GamePiece(corners));
      }
      gamePieces.clear();
    }
  }

  default void updateInputs(GamePieceVisionIOInputs inputs) {}
}
