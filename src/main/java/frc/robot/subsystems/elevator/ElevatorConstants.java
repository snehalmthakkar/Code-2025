package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;

public final class ElevatorConstants {
    private ElevatorConstants() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    public static final Distance MIN_HEIGHT = Inches.of(41.50);
    public static final Distance MAX_HEIGHT = Inches.of(73);
    public static final Distance TOLERANCE = Inches.of(0.75);

    public static final Mass ELEVATOR_MASS = Pounds.of(20.0);

    public static final double MOTOR_GEAR_REDUCTION = 50.0 / 12.0;
    public static final Distance SPOOL_DIAMETER = Inches.of(2.148);
    public static final double SENSOR_MECHANISM_RATIO = MOTOR_GEAR_REDUCTION / SPOOL_DIAMETER.times(Math.PI).in(Meters);

    public static final int LEFT_MOTOR_ID = 5;
    public static final int RIGHT_MOTOR_ID = 4;

    public static final InvertedValue LEFT_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue RIGHT_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive;

    public static final SlotConfigs emptySlot = new SlotConfigs()
        .withKP(6)
        .withKI(0.3)
        .withKD(1)
        .withKG(0.8195)
        .withKV(3.13)
        .withKA(0.079)
        .withKS(0.2605)
        .withGravityType(GravityTypeValue.Elevator_Static)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);
    public static final SlotConfigs coralSlot = new SlotConfigs()
        .withKP(6)
        .withKI(0.3)
        .withKD(1)
        .withKG(0.8195)
        .withKV(3.13)
        .withKA(0.079)
        .withKS(0.2605)
        .withGravityType(GravityTypeValue.Elevator_Static)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);
    public static final SlotConfigs algaeSlot = new SlotConfigs()
        .withKP(6)
        .withKI(0.3)
        .withKD(1)
        .withKG(0.8195)
        .withKV(3.13)
        .withKA(0.079)
        .withKS(0.2605)
        .withGravityType(GravityTypeValue.Elevator_Static)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);
    public static final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(2.25)
            .withMotionMagicAcceleration(4);
    public static final CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(90)
            .withSupplyCurrentLimitEnable(true);
    public static final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake);

    // KG - KS correct
    // KG + KS too low


    public static final int DIO_FLOOR_PORT = 1;
    public static final int DIO_CEILING_PORT = 0;

    public static final Voltage KG = Volts.of(0.8195);
    public static final Voltage FINE_CONTROL_UP = KG.plus(Volts.of(1));
    public static final Voltage FINE_CONTROL_DOWN = KG.minus(Volts.of(1));
}
