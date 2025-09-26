package frc.robot.subsystems.intake.sensor;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.intake.IntakeConstants;

public class IntakeSensorSim implements IntakeSensor {
    private double lastDetectionTimestamp;

    @Override
    public boolean detectsCoral() {
        return Timer.getFPGATimestamp() - lastDetectionTimestamp < IntakeConstants.Simulation.sensorDetectionTime.in(Seconds);
    }

    @Override
    public void simulateDetection() {
        lastDetectionTimestamp = Timer.getFPGATimestamp();
    }
}
