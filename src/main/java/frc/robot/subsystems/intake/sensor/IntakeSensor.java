package frc.robot.subsystems.intake.sensor;

import static edu.wpi.first.units.Units.Seconds;

import com.team6962.lib.digitalsensor.DigitalSensor;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.intake.IntakeConstants;

public class IntakeSensor extends DigitalSensor {
    public IntakeSensor(int channel, DigitalSensor.Wiring wiring) {
        super(channel, wiring);
    }

    private double lastDetectionTimestamp;

    @Override
    public void simulationPeriodic() {
        setTriggeredInSimulation(Timer.getFPGATimestamp() - lastDetectionTimestamp < IntakeConstants.Simulation.sensorDetectionTime.in(Seconds));
    }

    public void simulateDetection() {
        lastDetectionTimestamp = Timer.getFPGATimestamp();
    }
}
