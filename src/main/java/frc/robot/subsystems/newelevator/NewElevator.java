package frc.robot.subsystems.newelevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.team6962.lib.utils.CTREUtils;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NewElevator extends SubsystemBase {
    private TalonFX leftMotor;
    private TalonFX rightMotor;

    private DigitalInput bottomLimitSwitch;
    private DigitalInput topLimitSwitch;

    public NewElevator() {
        leftMotor = new TalonFX(NewElevatorConstants.LEFT_MOTOR_ID); // Replace with actual CAN ID
        rightMotor = new TalonFX(NewElevatorConstants.RIGHT_MOTOR_ID); // Replace with actual CAN ID

        bottomLimitSwitch = new DigitalInput(NewElevatorConstants.DIO_FLOOR_PORT); // Replace with actual DIO port
        topLimitSwitch = new DigitalInput(NewElevatorConstants.DIO_CEILING_PORT); // Replace with actual DIO port

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration()
        .withSlot0(NewElevatorConstants.slot0Configs)
        .withMotionMagic(NewElevatorConstants.motionMagicConfigs)
        .withCurrentLimits(NewElevatorConstants.currentLimitsConfigs)
        .withMotorOutput(NewElevatorConstants.motorOutputConfigs)
        .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(NewElevatorConstants.SENSOR_MECHANISM_RATIO)); // Assuming 1:1 ratio, adjust if necessary

        talonFXConfiguration.MotorOutput.Inverted = NewElevatorConstants.LEFT_MOTOR_INVERTED_VALUE; // Set inversion for left motor
        CTREUtils.check(leftMotor.getConfigurator().apply(talonFXConfiguration));

        talonFXConfiguration.MotorOutput.Inverted = NewElevatorConstants.RIGHT_MOTOR_INVERTED_VALUE; // Set inversion for left motor
        CTREUtils.check(rightMotor.getConfigurator().apply(talonFXConfiguration));
    }

    public Distance getPosition() {
        return Meters.of(CTREUtils.unwrap(leftMotor.getPosition()).in(Rotations));
    }

    public LinearVelocity getVelocity() {
        return MetersPerSecond.of(CTREUtils.unwrap(leftMotor.getVelocity()).in(RotationsPerSecond));
    }

    public Command moveToPosition(Distance position) {
        return runOnce(() -> {
            ControlRequest controlRequest = new PositionVoltage(position.in(Meters))
                .withLimitForwardMotion(false);

            leftMotor.setControl(controlRequest);
            rightMotor.setControl(controlRequest);
        });
    }
    
    @Override
    public void periodic() {
        if (!bottomLimitSwitch.get()) {
            leftMotor.setPosition(NewElevatorConstants.MIN_HEIGHT.in(Meters));
            rightMotor.setPosition(NewElevatorConstants.MIN_HEIGHT.in(Meters));
        }
    }
}
