package frc.robot.subsystems.intake;

import com.team6962.lib.digitalsensor.BeamBreak;
import com.team6962.lib.telemetry.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.indexer.Indexer;
import frc.robot.subsystems.manipulator.grabber.Grabber;

public class IntakeSensors extends SubsystemBase {
    private final BeamBreak intakeSensor;
    private final BeamBreak indexerSensor;
    private final Indexer indexer;
    private final Grabber grabber;

    private CoralLocation location;

    public static enum CoralLocation {
        OUTSIDE("Outside of Intake and Indexer"),
        INTAKE("Intake"),
        TRANSFER_TO_INDEXER("Transferring from Intake to Indexer"),
        INDEXER("Indexer"),
        TRANSFER_TO_MANIPULATOR("Transferring to Manipulator");

        public final String name;

        private CoralLocation(String name) {
            this.name = name;
        }
    }

    public IntakeSensors(Grabber grabber, Indexer indexer) {
        intakeSensor = new BeamBreak(IntakeConstants.intakeSensorChannel, IntakeConstants.intakeSensorWiring, IntakeConstants.Simulation.intakeSensorDetectionTime);
        indexerSensor = new BeamBreak(IntakeConstants.indexerSensorChannel, IntakeConstants.indexerSensorWiring, IntakeConstants.Simulation.indexerSensorDetectionTime);
        this.grabber = grabber;
        this.indexer = indexer;
        this.location = CoralLocation.OUTSIDE;

        Logger.logDigitalSensor("Intake/intakeCoralDetected", intakeSensor);
        Logger.logDigitalSensor("Intake/indexerCoralDetected", indexerSensor);
        Logger.logString("Intake/coralLocation", () -> location.name);
    }

    public CoralLocation getCoralLocation() {
        return location;
    }

    public void simIntakeCoral() {
        if (RobotBase.isReal()) return;
        location = CoralLocation.INTAKE;
    }

    public void simIndexCoral() {
        if (RobotBase.isReal()) return;
        location = CoralLocation.INDEXER;
    }

    public void simTransferCoral() {
        if (RobotBase.isReal()) return;
        location = CoralLocation.TRANSFER_TO_MANIPULATOR;
    }

    public void simDropCoral() {
        if (RobotBase.isReal()) return;
        location = CoralLocation.OUTSIDE;
    }

    @Override
    public void periodic() {
        if (location == CoralLocation.INDEXER && !grabber.isCoralClear()) {
            location = CoralLocation.TRANSFER_TO_MANIPULATOR;
        }

        if (location == CoralLocation.INDEXER && !indexerSensor.isTriggered()) {
            location = CoralLocation.OUTSIDE;
        }

        if (location == CoralLocation.TRANSFER_TO_MANIPULATOR && grabber.isCoralClear() && grabber.hasCoral()) {
            location = CoralLocation.OUTSIDE;
        }

        if (location == CoralLocation.OUTSIDE && intakeSensor.isTriggered()) {
            location = CoralLocation.INTAKE;
        }

        if (location == CoralLocation.INDEXER && !intakeSensor.isTriggered() && indexer.isDropping()) {
            location = CoralLocation.OUTSIDE;
        }

        if (location == CoralLocation.INTAKE && !intakeSensor.isTriggered()) {
            location = CoralLocation.TRANSFER_TO_INDEXER;
        }

        if (location == CoralLocation.TRANSFER_TO_INDEXER && indexerSensor.isTriggered()) {
            location = CoralLocation.INDEXER;
        }
    }
}
