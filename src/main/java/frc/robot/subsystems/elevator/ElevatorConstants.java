package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public final class ElevatorConstants {
    private ElevatorConstants() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    public static final Distance MIN_HEIGHT = Inches.of(41.50);
    public static final Distance MAX_HEIGHT = Inches.of(70.65);
    public static final Distance TOLERANCE = Inches.of(0.5);

    public static final Mass ELEVATOR_MASS = Pounds.of(20.0);

    public static final double MOTOR_GEAR_REDUCTION = 50.0 / 12.0;
    public static final Distance SPOOL_DIAMETER = Inches.of(2.148);
    public static final double SENSOR_MECHANISM_RATIO = MOTOR_GEAR_REDUCTION / SPOOL_DIAMETER.times(Math.PI).in(Meters);

    public static final int LEFT_MOTOR_ID = 5;
    public static final int RIGHT_MOTOR_ID = 4;

    public static final InvertedValue LEFT_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue RIGHT_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive;

    public static final Slot0Configs slot0Configs = new Slot0Configs()
            .withKP(600)
            .withKI(40)
            .withKD(150)
            .withKS(21)
            .withKG(53)
            .withKA(4.85)
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);
    public static final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(4)
            .withMotionMagicAcceleration(10);
    public static final CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(60)
            .withSupplyCurrentLimitEnable(true);
    public static final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake);

    public static final int DIO_FLOOR_PORT = 1;
    public static final int DIO_CEILING_PORT = 0;

    public static final Current FINE_CONTROL_CURRENT = Amps.of(37.5); // 22-53
}
