package frc.robot.subsystems.newelevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.telemetry.MechanismLogger;
import com.team6962.lib.utils.CTREUtils;
import com.team6962.lib.utils.MeasureMath;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.software.MathUtils;

public class NewElevator extends SubsystemBase {
    protected TalonFX leftMotor;
    protected TalonFX rightMotor;

    protected DigitalInput bottomLimitSwitch;
    protected DigitalInput topLimitSwitch;

    protected PositionVoltage positionControl;

    protected boolean elevatorZeroed = false;

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

        Logger.logMeasure("NewElevator/position", this::getPosition);
        Logger.logMeasure("NewElevator/velocity", this::getVelocity);
        Logger.logBoolean("NewElevator/topLimitSwitchTriggered", this::topLimitSwitchTriggered);
        Logger.logBoolean("NewElevator/bottomLimitSwitchTriggered", this::bottomLimitSwitchTriggered);
        Logger.logMeasure("NewElevator/supplyCurrentLeft", () -> CTREUtils.unwrap(leftMotor.getSupplyCurrent()));
        Logger.logMeasure("NewElevator/supplyCurrentRight", () -> CTREUtils.unwrap(rightMotor.getSupplyCurrent()));
        Logger.logMeasure("NewElevator/statorCurrentLeft", () -> CTREUtils.unwrap(leftMotor.getStatorCurrent()));
        Logger.logMeasure("NewElevator/statorCurrentRight", () -> CTREUtils.unwrap(rightMotor.getStatorCurrent()));

        MechanismRoot2d root = MechanismLogger.getRoot("Elevator", -6, 39.457496 - NewElevatorConstants.MIN_HEIGHT.in(Inches) + 1);
        MechanismLigament2d ligament = MechanismLogger.getLigament("Elevator Top", NewElevatorConstants.MIN_HEIGHT.in(Inches), 90, 6, new Color8Bit(255, 0, 0));
        
        root.append(ligament);

        MechanismLogger.addDynamicLength(ligament, () -> getPosition().minus(Inches.of(1)));

        MechanismRoot2d carriage = MechanismLogger.getRoot("Manipulator Pivot");

        MechanismLogger.addDyanmicPosition(
            carriage,
            () -> Inches.of(-9.495),
            () -> Inches.of(MathUtils.map(getPosition().in(Inches), NewElevatorConstants.MIN_HEIGHT.in(Inches), NewElevatorConstants.MAX_HEIGHT.in(Inches), 9.825, 67.717005))
        );
    }

    public Distance getPosition() {
        return Meters.of(CTREUtils.unwrap(leftMotor.getPosition()).in(Rotations));
    }

    public LinearVelocity getVelocity() {
        return MetersPerSecond.of(CTREUtils.unwrap(leftMotor.getVelocity()).in(RotationsPerSecond));
    }

    protected boolean topLimitSwitchTriggered() {
        return topLimitSwitch.get();
    }

    protected boolean bottomLimitSwitchTriggered() {
        return !bottomLimitSwitch.get();
    }

    private void startPositionControl(Distance position) {
        Distance clampedPosition = MeasureMath.clamp(position, NewElevatorConstants.MIN_HEIGHT, NewElevatorConstants.MAX_HEIGHT);
        
        PositionVoltage controlRequest = new PositionVoltage(clampedPosition.in(Meters))
            .withLimitForwardMotion(topLimitSwitchTriggered() || !elevatorZeroed).withLimitReverseMotion(bottomLimitSwitchTriggered());

        positionControl = controlRequest;
        leftMotor.setControl(controlRequest);
        rightMotor.setControl(controlRequest);
    }

    public Command moveToPosition(Distance position) {
        return startEnd(() -> startPositionControl(position), () -> startPositionControl(getPosition()));
    }
    
    @Override
    public void periodic() {
        if (RobotState.isDisabled() || positionControl == null) {
            startPositionControl(getPosition());
        }

        positionControl = positionControl
            .withLimitForwardMotion(topLimitSwitchTriggered() || !elevatorZeroed)
            .withLimitReverseMotion(bottomLimitSwitchTriggered());
        
        leftMotor.setControl(positionControl);
        rightMotor.setControl(positionControl);

        if (bottomLimitSwitchTriggered()) {
            leftMotor.setPosition(NewElevatorConstants.MIN_HEIGHT.in(Meters));
            rightMotor.setPosition(NewElevatorConstants.MIN_HEIGHT.in(Meters));

            elevatorZeroed = true;
        }
    }
}
