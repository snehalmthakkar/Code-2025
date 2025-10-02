package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.team6962.lib.digitalsensor.DigitalSensor;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

public final class IntakeConstants {
    private IntakeConstants() {}

    public static final int pivotMotorID = 2;
    public static final int absoluteEncoderID = 1;
    public static final int rollerMotorID = 3;
    public static final String canBus = "drivetrain";

    public static final int intakeSensorChannel = 4;
    public static final DigitalSensor.Wiring intakeSensorWiring = DigitalSensor.Wiring.NormallyOpen;

    public static final TalonFXConfiguration pivotMotorConfiguration = new TalonFXConfiguration()
        .withSlot0(
            new Slot0Configs()
                .withKP(72)
                .withKD(27)
                .withKV(0)
                .withKA(0)
                .withKS(4.2)
                .withKG(0) // DO NOT USE
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
        )
        .withTorqueCurrent(
            new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(Amps.of(60))
                .withPeakReverseTorqueCurrent(Amps.of(-60))
        )
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
        );
    public static final Current kG = Amps.of(11);
    public static final double pivotSensorToMechanism = 2.25;
    public static final double pivotRotorToSensor = 24.0; // 49 - 15 amps - I = Kg * cos(-1 deg) => 15 / 0.9998476952 = Kg

    public static final Angle absoluteEncoderOffset = Degrees.of(47.7);
    public static final Angle centerOfMassAngularOffset = Degrees.of(-40);
    public static final SensorDirectionValue absoluteEncoderDirection = SensorDirectionValue.Clockwise_Positive;
    public static final Angle absoluteEncoderDiscontinuityPoint = Degrees.of(10);

    public static final Angle pivotMinAngle = Degrees.of(21);
    public static final Angle pivotMaxAngle = Degrees.of(157);

    public static final Angle pivotIntakeAngle = Degrees.of(21);
    public static final Angle pivotStowAngle = Degrees.of(150);
    public static final Angle pivotVerticalAngle = Degrees.of(50);

    public static final Angle pivotTolerance = Degrees.of(4);

    public static final TalonFXConfiguration rollerMotorConfiguration = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
            );
    public static final Voltage rollerIntakeVoltage = Volts.of(12);
    public static final Voltage rollerDropVoltage = Volts.of(-12);

    public static final Time delayAfterIntake = Seconds.of(0);

    public static final int indexerMotorId = 1;

    public static final TalonFXConfiguration indexerMotorConfiguration = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
        );

    public static final int indexerSensorChannel = 6;
    public static final DigitalSensor.Wiring indexerSensorWiring = DigitalSensor.Wiring.NormallyOpen;

    public static final Voltage indexerIntakeVoltage = Volts.of(8);
    public static final Voltage indexerDropVoltage = Volts.of(-10);

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
