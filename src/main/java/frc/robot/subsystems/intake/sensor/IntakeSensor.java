package frc.robot.subsystems.intake.sensor;

public interface IntakeSensor {
    public boolean detectsCoral();
    public void simulateDetection();

    public static enum Wiring {
        NormallyOpen(false),
        NormallyClosed(true);

        private boolean detectingValue;

        private Wiring(boolean detectingValue) {
            this.detectingValue = detectingValue;
        }

        public boolean getValueWhenDetecting() {
            return detectingValue;
        }
    }
}
