package frc.robot.vision.field;

import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionField extends SubsystemBase implements TrackingField {
    public static double CONFIDENCE_DECAY = 0.01;
    private static double ONE_MINUS_CONFIDENCE_DECAY = 1.0 - CONFIDENCE_DECAY;

    private List<GamePiece> gamePieces = new LinkedList<>();

    /**
     * Adds a detected game piece to the field data.
     * @param detectedGamePiece The detected game piece to add.
     */
    @Override
    public void addDetection(GamePiece detectedGamePiece) {
        double maxSimilarity = 0;
        GamePiece mostSimilarPiece = null;

        for (GamePiece gamePiece : gamePieces) {
            double similarity = detectedGamePiece.getSimilarityTo(gamePiece);

            if (similarity > maxSimilarity) {
                mostSimilarPiece = gamePiece;
                maxSimilarity = similarity;
            }
        }

        GamePiece gamePiece = detectedGamePiece;

        if (mostSimilarPiece != null) {
            gamePieces.remove(mostSimilarPiece);

            mostSimilarPiece.updateFrom(detectedGamePiece);

            gamePiece = mostSimilarPiece;
        }

        addGamePiece(gamePiece, gamePieces);
    }
    
    private static void addGamePiece(GamePiece newGamePiece, List<GamePiece> list) {
        int index = 0;
        for (GamePiece gamePiece : list) {
            if (gamePiece.getConfidence() < newGamePiece.getConfidence()) {
                list.add(index, newGamePiece);
            }

            index++;
        }
    }

    /**
     * Gets the list of game pieces currently tracked on the field, ordered from
     * highest confidence to lowest confidence.
     * @return List of game pieces currently tracked on the field.
     */
    @Override
    public List<GamePiece> getGamePieces() {
        return gamePieces;
    }

    @Override
    public void periodic() {
        Iterator<GamePiece> iterator = gamePieces.iterator();
        while (iterator.hasNext()) {
            GamePiece gamePiece = iterator.next();
            gamePiece.setConfidence(gamePiece.getConfidence() * ONE_MINUS_CONFIDENCE_DECAY);

            if (gamePiece.getConfidence() < 0.1) {
                iterator.remove();
            }
        }
    }
}
