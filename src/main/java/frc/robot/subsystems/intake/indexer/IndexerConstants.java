package frc.robot.subsystems.intake.indexer;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.team6962.lib.digitalsensor.DigitalSensor;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

public final class IndexerConstants {
    private IndexerConstants() {}

    public static int motorID = 8;
    public static String canBus = "drivetrain";

    public static TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

    public static int sensorChannel = 6;
    public static DigitalSensor.Wiring sensorWiring = DigitalSensor.Wiring.NormallyOpen;
    public static Time sensorSimulatedDetectionTime = Seconds.of(0.5);

    public static Voltage intakeVoltage = Volts.of(12);
    public static Voltage dropVoltage = Volts.of(-12);

    public static Time dropExtraRunTime = Seconds.of(0.2);
}
