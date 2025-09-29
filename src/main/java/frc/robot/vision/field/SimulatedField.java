package frc.robot.vision.field;

import java.util.ArrayList;
import java.util.List;

public class SimulatedField implements TrackingField {
    private List<GamePiece> gamePieces = new ArrayList<>();

    @Override
    public void addDetection(GamePiece detectedGamePiece) {
        // Does nothing
    }

    @Override
    public List<GamePiece> getGamePieces() {
        return gamePieces;
    }

    public void setGamePieces(List<GamePiece> gamePieces) {
        this.gamePieces = gamePieces;
    }
}
