package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.intake.sensor.IntakeSensor;

public final class IntakeConstants {
    private IntakeConstants() {}

    public static final int pivotMotorID = 1;
    public static final int absoluteEncoderID = 2;
    public static final int rollerMotorID = 3;
    public static final String canBus = "rio";

    public static final int sensorChannel = 0;
    public static final IntakeSensor.Wiring sensorWiring = IntakeSensor.Wiring.NormallyOpen;

    public static final TalonFXConfiguration pivotMotorConfiguration = new TalonFXConfiguration();
    public static final double pivotGearReduction = 1.0;

    public static final Angle absoluteEncoderOffset = Degrees.of(0);
    public static final Angle centerOfMassAngularOffset = Degrees.of(0);
    public static final SensorDirectionValue absoluteEncoderDirection = SensorDirectionValue.CounterClockwise_Positive;

    public static final Angle minPivotAngle = Degrees.of(0);
    public static final Angle maxPivotAngle = Degrees.of(90);

    public static final Angle pivotIntakeAngle = Degrees.of(5);
    public static final Angle pivotStowAngle = Degrees.of(85);

    public static final Angle pivotTolerance = Degrees.of(2);

    public static final TalonFXConfiguration rollerMotorConfiguration = new TalonFXConfiguration();
    public static final Voltage rollerIntakeVoltage = Volts.of(12).times(Percent.of(50));

    public static final Time delayAfterIntake = Seconds.of(0.25);

    public static final class Simulation {
        private Simulation() {}
        
        public static final MomentOfInertia rollersMOI = KilogramSquareMeters.of(0.0001);
        public static final double rollersReduction = 1.0;
        public static final DCMotor rollersMotor = DCMotor.getKrakenX60Foc(1);

        public static final DCMotor pivotMotor = DCMotor.getKrakenX60Foc(1);
        public static final MomentOfInertia pivotMomentOfInertia = KilogramSquareMeters.of(0.01);
        public static final Distance pivotCenterOfMassDistance = Inches.of(12);
        public static final Angle pivotStartAngle = minPivotAngle;

        public static final Time sensorDetectionTime = Seconds.of(0.125);
    }
}
