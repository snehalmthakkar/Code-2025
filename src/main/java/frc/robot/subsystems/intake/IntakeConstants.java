package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.team6962.lib.digitalsensor.DigitalSensor;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

public final class IntakeConstants {
    private IntakeConstants() {}

    public static final int pivotMotorID = 1;
    public static final int absoluteEncoderID = 2;
    public static final int rollerMotorID = 3;
    public static final String canBus = "rio";

    public static final int intakeSensorChannel = 4;
    public static final DigitalSensor.Wiring intakeSensorWiring = DigitalSensor.Wiring.NormallyOpen;

    public static final TalonFXConfiguration pivotMotorConfiguration = new TalonFXConfiguration()
        .withSlot0(
            new Slot0Configs()
                .withKP(100)
                .withKV(6.70)
                .withKA(0.04)
                .withKG(0.24)
                .withGravityType(GravityTypeValue.Arm_Cosine)
        );
    public static final double pivotSensorToMechanism = 24.0;
    public static final double pivotRotorToSensor = 2.25;

    public static final Angle absoluteEncoderOffset = Degrees.of(0);
    public static final Angle centerOfMassAngularOffset = Degrees.of(0);
    public static final SensorDirectionValue absoluteEncoderDirection = SensorDirectionValue.CounterClockwise_Positive;

    public static final Angle pivotMinAngle = Degrees.of(-30);
    public static final Angle pivotMaxAngle = Degrees.of(90);

    public static final Angle pivotIntakeAngle = Degrees.of(-30);
    public static final Angle pivotStowAngle = Degrees.of(85);

    public static final Angle pivotTolerance = Degrees.of(2);

    public static final TalonFXConfiguration rollerMotorConfiguration = new TalonFXConfiguration();
    public static final Voltage rollerIntakeVoltage = Volts.of(12).times(Percent.of(50));

    public static final Time delayAfterIntake = Seconds.of(0);

    public static final int indexerMotorId = 8;

    public static final TalonFXConfiguration indexerMotorConfiguration = new TalonFXConfiguration();

    public static final int indexerSensorChannel = 8;
    public static final DigitalSensor.Wiring indexerSensorWiring = DigitalSensor.Wiring.NormallyOpen;

    public static final Voltage indexerIntakeVoltage = Volts.of(12);
    public static final Voltage indexerDropVoltage = Volts.of(-12);

    public static final Time indexerDropExtraTime = Seconds.of(0.2);

    public static final class Simulation {
        private Simulation() {}

        public static final DCMotor pivotMotor = DCMotor.getKrakenX60Foc(1);
        public static final MomentOfInertia pivotMomentOfInertia = KilogramSquareMeters.of(Pounds.of(10).in(Kilograms) * Inches.of(9.47).in(Meters) * Inches.of(9.47).in(Meters));
        public static final Distance pivotCenterOfMassDistance = Inches.of(9.47);
        public static final Angle pivotStartAngle = pivotStowAngle;

        public static final Time intakeSensorDetectionTime = Seconds.of(0.125);
        public static final Time indexerSensorDetectionTime = Seconds.of(0.125);
    }
}
