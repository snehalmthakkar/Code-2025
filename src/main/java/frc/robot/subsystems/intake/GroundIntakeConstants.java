package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;

public final class GroundIntakeConstants {
    private GroundIntakeConstants() {}

    public static final int motorID = 1;
    public static final int absoluteEncoderID = 2;
    public static final String canBus = "rio";

    public static final TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
    public static final double gearReduction = 1.0;

    public static final Angle absoluteEncoderOffset = Degrees.of(0);
    public static final Angle centerOfMassAngularOffset = Degrees.of(0);
    public static final SensorDirectionValue absoluteEncoderDirection = SensorDirectionValue.CounterClockwise_Positive;

    public static final Angle minAngle = Degrees.of(0);
    public static final Angle maxAngle = Degrees.of(90);
}
