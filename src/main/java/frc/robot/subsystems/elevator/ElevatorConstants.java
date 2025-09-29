package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;

public final class ElevatorConstants {
    private ElevatorConstants() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    public static final Distance MIN_HEIGHT = Inches.of(41.50);
    public static final Distance MAX_HEIGHT = Inches.of(70.65);
    public static final Distance TOLERANCE = Inches.of(0.5);

    public static final Mass ELEVATOR_MASS = Pounds.of(20.0);

    public static final double MOTOR_GEAR_REDUCTION = 4.375;
    public static final Distance SPOOL_DIAMETER = Inches.of(1.0); // change
    public static final double SENSOR_MECHANISM_RATIO = MOTOR_GEAR_REDUCTION / SPOOL_DIAMETER.times(Math.PI).in(Meters); // Adjust if necessary

    public static final int LEFT_MOTOR_ID = 5; // Replace with actual CAN ID
    public static final int RIGHT_MOTOR_ID = 4; // Replace with actual CAN ID

    public static final InvertedValue LEFT_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive; // Set inversion for left motor
    public static final InvertedValue RIGHT_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive; // Set inversion for left motor

    public static final Slot0Configs slot0Configs = new Slot0Configs()
            .withKP(0) // Replace with actual P value
            .withKI(0) // Replace with actual I value
            .withKD(0) // Replace with actual D value
            .withKG(0)
            .withGravityType(GravityTypeValue.Elevator_Static);
    public static final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(1500) // Replace with actual cruise velocity
            .withMotionMagicAcceleration(600); // Replace with actual acceleration
    public static final CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(80) // Replace with actual peak current limit
            .withSupplyCurrentLimitEnable(true);
    public static final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake);

    public static final int DIO_FLOOR_PORT = 1;
    public static final int DIO_CEILING_PORT = 0;

    public static final Voltage FINE_CONTROL_VOLTAGE = Volts.of(0.5);
}
