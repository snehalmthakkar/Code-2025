/**
 * ElevatorConstants.java
 *
 * This file defines the configuration constants for the elevator subsystem on the robot.
 * All physical dimensions, motor IDs, control slots, and hardware interface IDs are defined here.
 * These constants are used throughout the elevator control code to avoid duplication, aid maintainability,
 * and provide a single point of configuration for subsystem tuning.
 */

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

/**
 * Utility class holding all configuration constants for the Elevator subsystem.
 * This class cannot be instantiated.
 */
public final class ElevatorConstants {
    /**
     * Private constructor to prevent instantiation.
     * This is a utility class and should only be accessed in a static way.
     */
    private ElevatorConstants() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /** Minimum height of elevator in inches. */
    public static final Distance MIN_HEIGHT = Inches.of(41.50);
    /** Maximum height of elevator in inches. */
    public static final Distance MAX_HEIGHT = Inches.of(73);
    /** Allowable tolerance for elevator position in inches. */
    public static final Distance TOLERANCE = Inches.of(0.75);

    /** Mass of the elevator in pounds. */
    public static final Mass ELEVATOR_MASS = Pounds.of(20.0);

    /** Motor gear reduction ratio. */
    public static final double MOTOR_GEAR_REDUCTION = 50.0 / 12.0;
    /** Diameter of the elevator's spool in inches. */
    public static final Distance SPOOL_DIAMETER = Inches.of(2.148);
    /** Ratio for converting sensor measurements to mechanism position. */
    public static final double SENSOR_MECHANISM_RATIO = MOTOR_GEAR_REDUCTION / SPOOL_DIAMETER.times(Math.PI).in(Meters);

    /** CAN ID for the left elevator motor controller. */
    public static final int LEFT_MOTOR_ID = 5;
    /** CAN ID for the right elevator motor controller. */
    public static final int RIGHT_MOTOR_ID = 4;

    /** Inversion setting for left elevator motor. */
    public static final InvertedValue LEFT_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    /** Inversion setting for right elevator motor. */
    public static final InvertedValue RIGHT_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive;

    /** PID and feedforward slot configuration for empty elevator. */
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
    /** PID and feedforward slot configuration for coral-loaded elevator. */
    public static final SlotConfigs coralSlot = new SlotConfigs()
        .withKP(6)
        .withKI(0.3)
        .withKD(1)
        .withKG(0.93)
        .withKV(3.13)
        .withKA(0.0795)
        .withKS(0.2605)
        .withGravityType(GravityTypeValue.Elevator_Static)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);
    /** PID and feedforward slot configuration for algae-loaded elevator. */
    public static final SlotConfigs algaeSlot = new SlotConfigs()
        .withKP(6)
        .withKI(0.3)
        .withKD(1)
        .withKG(1.0)
        .withKV(3.13)
        .withKA(0.081)
        .withKS(0.2605)
        .withGravityType(GravityTypeValue.Elevator_Static)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    /** Motion Magic configuration for smooth elevator movement. */
    public static final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(2)
            .withMotionMagicAcceleration(3);

    /** Current limit configuration for elevator motors. */
    public static final CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(90)
            .withSupplyCurrentLimitEnable(true);

    /** Motor output configuration to use brake mode. */
    public static final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake);

    // KG - KS correct
    // KG + KS too low

    /** DIO port number for the floor limit switch. */
    public static final int DIO_FLOOR_PORT = 1;
    /** DIO port number for the ceiling limit switch. */
    public static final int DIO_CEILING_PORT = 0;

    /** Static gravity compensation voltage (KG). */
    public static final Voltage KG = Volts.of(0.8195);
    /** Fine control up voltage (KG + 1 V). */
    public static final Voltage FINE_CONTROL_UP = KG.plus(Volts.of(1));
    /** Fine control down voltage (KG - 1 V). */
    public static final Voltage FINE_CONTROL_DOWN = KG.minus(Volts.of(1));
}