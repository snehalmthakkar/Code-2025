package frc.robot.vision.field;

import java.util.List;

import com.team6962.lib.telemetry.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public interface TrackingField {
    public static TrackingField createInstance() {
        if (RobotBase.isSimulation()) {
            return new SimulatedField();
        } else {
            return new VisionField();
        }
    }

    /**
     * Adds a detected game piece to the field, merging it with existing
     * game pieces if they are close enough.
     * 
     * @param detectedGamePiece The detected game piece to add.
     */
    public void addDetection(GamePiece detectedGamePiece);

    /**
     * Gets the list of game pieces currently tracked on the field, ordered from
     * highest confidence to lowest confidence.
     * 
     * @return List of game pieces currently tracked on the field.
     */
    public List<GamePiece> getGamePieces();

    /**
     * Gets the list of pieces of coral currently tracked on the field, ordered
     * from highest confidence to lowest confidence.
     */
    public default List<Coral> getCoralPieces() {
        return getGamePieces().stream()
            .filter(piece -> piece instanceof Coral)
            .map(piece -> (Coral) piece)
            .toList();
    }

    /**
     * Gets the piece of coral that is most likely to exist.
     */
    public default Coral getMostLikelyCoral() {
        List<Coral> coral = getCoralPieces();
        if (coral.size() == 0) return null;
        return coral.get(0);
    }

    public default void startLogging() {
        Logger.addUpdate("TrackingField", () -> {
            Field2d field = Logger.getField();

            field.getObject("Coral").setPoses(
                getCoralPieces().stream()
                    .map(coral -> new Pose2d(coral.getTranslation(), coral.getOrientation() == Coral.Orientation.Vertical ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(90)))
                    .toList()
            );

            field.getObject("Other Game Pieces").setPoses(
                getGamePieces().stream()
                    .filter(gamePiece -> !(gamePiece instanceof Coral))
                    .map(gamePiece -> new Pose2d(gamePiece.getTranslation(), new Rotation2d()))
                    .toList()
            );
        });
    }

    public static abstract class GamePiece {
        private Translation2d translation;
        private double confidence;

        public void setTranslation(Translation2d translation) {
            this.translation = translation;
        }

        public GamePiece withTranslation(Translation2d translation) {
            setTranslation(translation);
            return this;
        }

        public void setConfidence(double confidence) {
            this.confidence = confidence;
        }

        public GamePiece withConfidence(double confidence) {
            setConfidence(confidence);
            return this;
        }

        public Translation2d getTranslation() {
            return translation;
        }

        public double getConfidence() {
            return this.confidence;
        }

        public void updateFrom(GamePiece update) {
            double targetConfidence = getConfidence();
            double updateConfidence = update.getConfidence();

            Translation2d updatedTranslation = translation.times(targetConfidence)
                .plus(update.getTranslation().times(updateConfidence))
                .div(targetConfidence + updateConfidence);
            
            setTranslation(updatedTranslation);
            setConfidence(Math.min(1.0, targetConfidence + updateConfidence));
        }

        public abstract double getSimilarityTo(GamePiece other);
    }

    public static class Coral extends GamePiece {
        public static enum Orientation { Vertical, Horizontal };

        private Orientation orientation = Orientation.Horizontal;

        public void setOrientation(Orientation orientation) {
            this.orientation = orientation;
        }
        
        @Override
        public Coral withTranslation(Translation2d translation) {
            setTranslation(translation);
            return this;
        }

        @Override
        public Coral withConfidence(double confidence) {
            setConfidence(confidence);
            return this;
        }
        
        public Coral withOrientation(Orientation orientation) {
            setOrientation(orientation);
            return this;
        }

        public Orientation getOrientation() {
            return orientation;
        }

        @Override
        public void updateFrom(GamePiece update) {
            if (!(update instanceof Coral)) return;
            super.updateFrom(update);
            setOrientation(((Coral) update).getOrientation());
        }

        @Override
        public double getSimilarityTo(GamePiece other) {
            if (!(other instanceof Coral)) return 0;
            return 1 - getTranslation().getDistance(other.translation) / Units.inchesToMeters(12);
        }
    }
}
