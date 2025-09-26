package com.team6962.lib.digitalsensor;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DigitalSensor extends SubsystemBase {
    private DigitalInput input;
    private Wiring wiring;
    private boolean triggeredSim;

    public DigitalSensor(int channel, Wiring wiring) {
        input = new DigitalInput(channel);
        this.wiring = wiring;
    }

    public boolean isTriggered() {
        if (RobotBase.isSimulation()) {
            return triggeredSim;
        }

        return input.get() == wiring.triggeredValue;
    }

    public boolean notTriggered() {
        return !isTriggered();
    }

    public void setTriggeredInSimulation(boolean triggered) {
        this.triggeredSim = triggered;
    }

    public static enum Wiring {
        NormallyOpen(false),
        NormallyClosed(true);

        private boolean triggeredValue;

        private Wiring(boolean triggeredValue) {
            this.triggeredValue = triggeredValue;
        }

        public boolean getValueWhenDetecting() {
            return triggeredValue;
        }
    }
}
