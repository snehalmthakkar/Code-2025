package frc.robot.subsystems.intake.sensor;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.intake.IntakeConstants;

public class IntakeSensorReal implements IntakeSensor {
    private final DigitalInput input = new DigitalInput(IntakeConstants.sensorChannel);

    @Override
    public boolean detectsCoral() {
        return input.get() == IntakeConstants.sensorWiring.getValueWhenDetecting();
    }

    @Override
    public void simulateDetection() {
        // Do nothing, because this is a real sensor
    }
}
