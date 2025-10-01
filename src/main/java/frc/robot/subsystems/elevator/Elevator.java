package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.telemetry.MechanismLogger;
import com.team6962.lib.utils.CTREUtils;
import com.team6962.lib.utils.MeasureMath;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.software.MathUtils;

public class Elevator extends SubsystemBase {
    protected TalonFX leftMotor;
    protected TalonFX rightMotor;

    protected DigitalInput bottomLimitSwitch;
    protected DigitalInput topLimitSwitch;

    protected MotionMagicVoltage positionControl;

    protected boolean elevatorZeroed = false;
    protected Distance targetPosition = Inches.of(0);
    private Distance commandTarget = Inches.of(0);

    private StatusSignal<Angle> positionSignal;
    private StatusSignal<AngularVelocity> velocitySignal;
    private StatusSignal<Current> supplyCurrentLeft;
    private StatusSignal<Current> supplyCurrentRight;
    private StatusSignal<Current> statorCurrentLeft;
    private StatusSignal<Current> statorCurrentRight;
    private StatusSignal<Voltage> motorVoltageLeft;
    private StatusSignal<Voltage> motorVoltageRight;
    private StatusSignal<Double> profilePositionSignal;

    public Elevator() {
        leftMotor = new TalonFX(ElevatorConstants.LEFT_MOTOR_ID, "drivetrain"); // Replace with actual CAN ID
        rightMotor = new TalonFX(ElevatorConstants.RIGHT_MOTOR_ID, "drivetrain"); // Replace with actual CAN ID

        bottomLimitSwitch = new DigitalInput(ElevatorConstants.DIO_FLOOR_PORT); // Replace with actual DIO port
        topLimitSwitch = new DigitalInput(ElevatorConstants.DIO_CEILING_PORT); // Replace with actual DIO port

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration()
            .withSlot0(ElevatorConstants.slot0Configs)
            .withSlot1(
                new Slot1Configs()
                    .withKP(ElevatorConstants.slot0Configs.kP)
                    .withKI(ElevatorConstants.slot0Configs.kI)
                    .withKD(ElevatorConstants.slot0Configs.kD)
                    .withKG(ElevatorConstants.slot0Configs.kG)
                    .withKS(ElevatorConstants.slot0Configs.kS)
                    .withGravityType(ElevatorConstants.slot0Configs.GravityType)
                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
            )
            .withMotionMagic(ElevatorConstants.motionMagicConfigs)
            .withCurrentLimits(ElevatorConstants.currentLimitsConfigs)
            .withMotorOutput(ElevatorConstants.motorOutputConfigs)
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(ElevatorConstants.SENSOR_MECHANISM_RATIO)); // Assuming 1:1 ratio, adjust if necessary

        talonFXConfiguration.MotorOutput.Inverted = ElevatorConstants.LEFT_MOTOR_INVERTED_VALUE; // Set inversion for left motor
        CTREUtils.check(leftMotor.getConfigurator().apply(talonFXConfiguration));

        talonFXConfiguration.MotorOutput.Inverted = ElevatorConstants.RIGHT_MOTOR_INVERTED_VALUE; // Set inversion for left motor
        CTREUtils.check(rightMotor.getConfigurator().apply(talonFXConfiguration));

        initStatusSignals();

        Logger.logMeasure("NewElevator/position", this::getPosition);
        Logger.logMeasure("NewElevator/velocity", this::getVelocity);
        Logger.logMeasure("NewElevator/commandTarget", () -> commandTarget);
        Logger.logBoolean("NewElevator/topLimitSwitchTriggered", this::topLimitSwitchTriggered);
        Logger.logBoolean("NewElevator/bottomLimitSwitchTriggered", this::bottomLimitSwitchTriggered);
        Logger.logMeasure("NewElevator/supplyCurrentLeft", () -> supplyCurrentLeft.getValue());
        Logger.logMeasure("NewElevator/supplyCurrentRight", () -> supplyCurrentRight.getValue());
        Logger.logMeasure("NewElevator/statorCurrentLeft", () -> statorCurrentLeft.getValue());
        Logger.logMeasure("NewElevator/statorCurrentRight", () -> statorCurrentRight.getValue());
        Logger.logMeasure("NewElevator/voltageLeft", () -> motorVoltageLeft.getValue());
        Logger.logMeasure("NewElevator/voltageRight", () -> motorVoltageRight.getValue());
        Logger.logMeasure("NewElevator/targetPosition", () -> targetPosition);
        Logger.logMeasure("NewElevator/profilePosition", () -> Meters.of(profilePositionSignal.getValue()));

        MechanismRoot2d root = MechanismLogger.getRoot("Elevator", -6, 39.457496 - ElevatorConstants.MIN_HEIGHT.in(Inches) + 1);
        MechanismLigament2d ligament = MechanismLogger.getLigament("Elevator Top", ElevatorConstants.MIN_HEIGHT.in(Inches), 90, 6, new Color8Bit(255, 0, 0));
        
        root.append(ligament);

        MechanismLogger.addDynamicLength(ligament, () -> getPosition().minus(Inches.of(1)));

        MechanismRoot2d carriage = MechanismLogger.getRoot("Manipulator Pivot");

        MechanismLogger.addDyanmicPosition(
            carriage,
            () -> Inches.of(-9.495),
            () -> Inches.of(MathUtils.map(getPosition().in(Inches), ElevatorConstants.MIN_HEIGHT.in(Inches), ElevatorConstants.MAX_HEIGHT.in(Inches), 9.825, 67.717005))
        );
    }

    private void initStatusSignals() {
        positionSignal = leftMotor.getPosition();
        velocitySignal = leftMotor.getVelocity();
        supplyCurrentLeft = leftMotor.getSupplyCurrent();
        supplyCurrentRight = rightMotor.getSupplyCurrent();
        statorCurrentLeft = leftMotor.getStatorCurrent();
        statorCurrentRight = rightMotor.getStatorCurrent();
        motorVoltageLeft = leftMotor.getMotorVoltage();
        motorVoltageRight = rightMotor.getMotorVoltage();
        profilePositionSignal = leftMotor.getClosedLoopReference();

        CTREUtils.check(BaseStatusSignal.setUpdateFrequencyForAll(
            Hertz.of(50),
            positionSignal, velocitySignal, supplyCurrentLeft, supplyCurrentRight,
            statorCurrentLeft, statorCurrentRight, motorVoltageLeft, motorVoltageRight,
            profilePositionSignal
        ));

        CTREUtils.check(ParentDevice.optimizeBusUtilizationForAll(leftMotor, rightMotor));
    }

    private void refreshStatusSignals() {
        BaseStatusSignal.refreshAll(
            positionSignal, velocitySignal, supplyCurrentLeft, supplyCurrentRight,
            statorCurrentLeft, statorCurrentRight, motorVoltageLeft, motorVoltageRight,
            profilePositionSignal
        );
    }

    public Distance getPosition() {
        return Meters.of(BaseStatusSignal.getLatencyCompensatedValueAsDouble(positionSignal, velocitySignal));
    }

    public LinearVelocity getVelocity() {
        return MetersPerSecond.of(velocitySignal.getValueAsDouble());
    }

    public boolean isNear(Distance position) {
        return getPosition().isNear(position, ElevatorConstants.TOLERANCE);
    }

    protected boolean topLimitSwitchTriggered() {
        return topLimitSwitch.get();
    }

    protected boolean bottomLimitSwitchTriggered() {
        return !bottomLimitSwitch.get();
    }

    private void startPositionControl(Distance position, boolean hold) {
        targetPosition = position;
        
        Distance clampedPosition = MeasureMath.clamp(position, ElevatorConstants.MIN_HEIGHT, ElevatorConstants.MAX_HEIGHT);

        MotionMagicVoltage controlRequest = new MotionMagicVoltage(clampedPosition.in(Meters))
            .withLimitForwardMotion(topLimitSwitchTriggered() || !elevatorZeroed).withLimitReverseMotion(bottomLimitSwitchTriggered());
        
        if (position.isNear(getPosition(), Inches.of(2))) controlRequest.Slot = 1;
        else controlRequest.Slot = 0;
        
        positionControl = controlRequest;

        leftMotor.setControl(controlRequest);
        rightMotor.setControl(controlRequest);
    }

    public Command moveToPosition(Distance position) {
        Command moveToCommand = new Command() {
            @Override
            public void initialize() {
                commandTarget = position;
                startPositionControl(position, false);
            }

            @Override
            public void end(boolean interrupted) {
                commandTarget = Inches.of(0);

                if (!isNear(position)) {
                    startPositionControl(getPosition(), true);
                } else {
                    startPositionControl(position, true);
                }
            }

            @Override
            public boolean isFinished() {
                return isNear(position);
            }
        };

        moveToCommand.addRequirements(this);

        return moveToCommand;
    }

    private Command fineControl(Voltage voltage) {
        return startEnd(
            () -> {
                leftMotor.setControl(new VoltageOut(voltage));
                rightMotor.setControl(new VoltageOut(voltage));
                positionControl = null;
            },
            () -> startPositionControl(getPosition(), true)
        );
    }

    public Command fineControlUp() {
        return fineControl(ElevatorConstants.FINE_CONTROL_UP);
    }

    public Command fineControlDown() {
        return fineControl(ElevatorConstants.FINE_CONTROL_DOWN);
    }
    
    @Override
    public void periodic() {
        refreshStatusSignals();

        if (bottomLimitSwitchTriggered() && !elevatorZeroed) {
            CTREUtils.check(leftMotor.setPosition(ElevatorConstants.MIN_HEIGHT.in(Meters)));
            CTREUtils.check(rightMotor.setPosition(ElevatorConstants.MIN_HEIGHT.in(Meters)));

            elevatorZeroed = true;

            startPositionControl(ElevatorConstants.MIN_HEIGHT, true);

            return;
        }
        
        if (RobotState.isDisabled()) {
            startPositionControl(getPosition(), true);
        }

        if (positionControl != null) {
            positionControl = positionControl
                .withLimitForwardMotion(topLimitSwitchTriggered() || !elevatorZeroed)
                .withLimitReverseMotion(bottomLimitSwitchTriggered());
            
            CTREUtils.check(leftMotor.setControl(positionControl));
            CTREUtils.check(rightMotor.setControl(positionControl));
        }
    }

    public Command stow() {
        return moveToPosition(Constants.ELEVATOR.STOW_HEIGHT);
    }

    public Command coralL1() {
        return moveToPosition(Constants.ELEVATOR.CORAL.L1_HEIGHT);
    }

    public Command coralL2() {
        return moveToPosition(Constants.ELEVATOR.CORAL.L2_HEIGHT);
    }

    public Command coralL3() {
        return moveToPosition(Constants.ELEVATOR.CORAL.L3_HEIGHT);
    }

    public Command coralL4() {
        return moveToPosition(Constants.ELEVATOR.CORAL.L4_HEIGHT);
    }

    public Command coralIntake() {
        return moveToPosition(Constants.ELEVATOR.CORAL.INTAKE_HEIGHT);
    }

    public Command algaeL2() {
        return moveToPosition(Constants.ELEVATOR.ALGAE.L2_HEIGHT);
    }

    public Command algaeL3() {
        return moveToPosition(Constants.ELEVATOR.ALGAE.L3_HEIGHT);
    }

    public Command algaeBarge() {
        return moveToPosition(Constants.ELEVATOR.ALGAE.BARGE_HEIGHT);
    }

    public Command ready() {
        return moveToPosition(Constants.ELEVATOR.AUTO.READY_HEIGHT);
    }

    public Command coral(int level) {
        return switch (level) {
            case 1 -> coralL1();
            case 2 -> coralL2();
            case 3 -> coralL3();
            case 4 -> coralL4();
            default -> throw new IllegalArgumentException("Invalid coral level: " + level);
        };
    }

    public Command algae(int level) {
        return switch (level) {
            case 2 -> algaeL2();
            case 3 -> algaeL3();
            default -> throw new IllegalArgumentException("Invalid algae level: " + level);
        };
    }

    public static Elevator create() {
        return RobotBase.isReal() ? new Elevator() : new SimElevator();
    }
}
