package com.team6962.lib.digitalsensor;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;

public class BeamBreak extends DigitalSensor {
    private double simulatedDetectionTime;

    public BeamBreak(int channel, DigitalSensor.Wiring wiring, Time simulatedDetectionTime) {
        super(channel, wiring);

        this.simulatedDetectionTime = simulatedDetectionTime.in(Seconds);
    }

    private double lastDetectionTimestamp;

    @Override
    public void simulationPeriodic() {
        setTriggeredInSimulation(Timer.getFPGATimestamp() - lastDetectionTimestamp < simulatedDetectionTime);
    }

    public void simulateDetection() {
        lastDetectionTimestamp = Timer.getFPGATimestamp();
    }
}
