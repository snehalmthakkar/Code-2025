package frc.robot.subsystems.elevator;

/**
 * Elevator Subsystem
 * 
 * This subsystem manages the robot's elevator mechanism, which is responsible for vertical movement
 * (raising and lowering the manipulator or other attached mechanisms) using two TalonFX motors.
 * It supports multiple motion profiles and levels for handling different game objectives (such as
 * picking up or scoring objects at different heights), incorporates limit switches for homing and safety,
 * and integrates with a Grabber subsystem to select appropriate control slots for different game pieces.
 * 
 * Key Features:
 * - Dual TalonFX motor control with configurable inversion and feedback
 * - Support for multiple control slots (PID profiles) and motion profiles
 * - Automatic zeroing using a bottom limit switch
 * - Command-based movement to predefined positions for game piece handling
 * - Safety via top and bottom limit switches
 * - Telemetry logging for key measurements and states
 * - Simulation compatibility via a SimElevator implementation
 * 
 * Usage:
 * Instantiate via the static {@link #create(Grabber)} factory method.
 * Use provided Commands for moving to target positions, fine control, and preset heights.
 */

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
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
import frc.robot.subsystems.manipulator.grabber.Grabber;
import frc.robot.util.software.MathUtils;

public class Elevator extends SubsystemBase {
    // Hardware
    protected TalonFX leftMotor;
    protected TalonFX rightMotor;
    protected DigitalInput bottomLimitSwitch; // True when at bottom
    protected DigitalInput topLimitSwitch;    // True when at top

    // Control state
    protected ControlRequest controlRequest;
    protected boolean elevatorZeroed = false; // Set true after homing
    protected Distance targetPosition = Inches.of(0); // Target for closed-loop control
    private Distance commandTarget = Inches.of(0);    // Target for current command

    // Status signals (for telemetry and control)
    private StatusSignal<Angle> positionSignal;
    private StatusSignal<AngularVelocity> velocitySignal;
    private StatusSignal<Current> supplyCurrentLeft;
    private StatusSignal<Current> supplyCurrentRight;
    private StatusSignal<Current> statorCurrentLeft;
    private StatusSignal<Current> statorCurrentRight;
    private StatusSignal<Voltage> motorVoltageLeft;
    private StatusSignal<Voltage> motorVoltageRight;
    private StatusSignal<Double> profilePositionSignal;

    private Grabber grabber;

    /**
     * Constructor.
     * Initializes hardware, configures TalonFX motors, sets up limit switches, telemetry logging,
     * and visualization for mechanism position.
     * 
     * @param grabber The Grabber subsystem (used to select PID profile slots depending on game piece)
     */
    public Elevator(Grabber grabber) {
        this.grabber = grabber;

        leftMotor = new TalonFX(ElevatorConstants.LEFT_MOTOR_ID, "drivetrain");
        rightMotor = new TalonFX(ElevatorConstants.RIGHT_MOTOR_ID, "drivetrain");

        bottomLimitSwitch = new DigitalInput(ElevatorConstants.DIO_FLOOR_PORT);
        topLimitSwitch = new DigitalInput(ElevatorConstants.DIO_CEILING_PORT);

        // Configure TalonFXs with appropriate slots, feedback, motion magic, limits, and inversion
        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration()
            .withSlot0(Slot0Configs.from(ElevatorConstants.emptySlot))
            .withSlot1(Slot1Configs.from(ElevatorConstants.coralSlot))
            .withSlot2(Slot2Configs.from(ElevatorConstants.algaeSlot))
            .withMotionMagic(ElevatorConstants.motionMagicConfigs)
            .withCurrentLimits(ElevatorConstants.currentLimitsConfigs)
            .withMotorOutput(ElevatorConstants.motorOutputConfigs)
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(ElevatorConstants.SENSOR_MECHANISM_RATIO));

        // Set inversion for left and right motors separately
        talonFXConfiguration.MotorOutput.Inverted = ElevatorConstants.LEFT_MOTOR_INVERTED_VALUE;
        CTREUtils.check(leftMotor.getConfigurator().apply(talonFXConfiguration));
        talonFXConfiguration.MotorOutput.Inverted = ElevatorConstants.RIGHT_MOTOR_INVERTED_VALUE;
        CTREUtils.check(rightMotor.getConfigurator().apply(talonFXConfiguration));

        initStatusSignals();

        // Set up telemetry logging for monitoring position, velocity, currents, voltages, etc.
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

        // Visualization setup for mechanism loggers (for dashboard or simulation display)
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

    /**
     * Initializes status signal objects for telemetry and fast data access.
     * Sets update frequency to 50Hz and optimizes bus usage.
     */
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

    /**
     * Refreshes all status signals from hardware.
     * Call periodically or before accessing cached signal values.
     */
    private void refreshStatusSignals() {
        BaseStatusSignal.refreshAll(
            positionSignal, velocitySignal, supplyCurrentLeft, supplyCurrentRight,
            statorCurrentLeft, statorCurrentRight, motorVoltageLeft, motorVoltageRight,
            profilePositionSignal
        );
    }

    /**
     * @return Current elevator position as a Distance (meters).
     */
    public Distance getPosition() {
        return Meters.of(BaseStatusSignal.getLatencyCompensatedValueAsDouble(positionSignal, velocitySignal));
    }

    /**
     * @return Current elevator velocity as a LinearVelocity (meters/second).
     */
    public LinearVelocity getVelocity() {
        return MetersPerSecond.of(velocitySignal.getValueAsDouble());
    }

    /**
     * Checks if the elevator is within a tolerance of a target position.
     * @param position Target position to compare to
     * @return true if elevator is near position, false otherwise
     */
    public boolean isNear(Distance position) {
        return getPosition().isNear(position, ElevatorConstants.TOLERANCE);
    }

    /**
     * @return true if the top limit switch is triggered (elevator at top)
     */
    protected boolean topLimitSwitchTriggered() {
        return topLimitSwitch.get();
    }

    /**
     * @return true if the bottom limit switch is triggered (elevator at bottom)
     */
    protected boolean bottomLimitSwitchTriggered() {
        return !bottomLimitSwitch.get();
    }

    /**
     * Starts closed-loop position control to move elevator to the specified position.
     * Selects PID slot based on Grabber state (coral, algae, or empty).
     * @param position Target position
     * @param hold If true, holds position after reaching target
     */
    private void startPositionControl(Distance position, boolean hold) {
        targetPosition = position;
        Distance clampedPosition = MeasureMath.clamp(position, ElevatorConstants.MIN_HEIGHT, ElevatorConstants.MAX_HEIGHT);

        MotionMagicVoltage controlRequest = new MotionMagicVoltage(clampedPosition.in(Meters));
        setLimits(controlRequest);

        // Select PID slot based on game piece
        if (grabber.hasCoral()) controlRequest.Slot = 1;
        else if (grabber.hasAlgae()) controlRequest.Slot = 2;
        else controlRequest.Slot = 0;

        this.controlRequest = controlRequest;

        leftMotor.setControl(controlRequest);
        rightMotor.setControl(controlRequest);
    }

    /**
     * Creates a Command to move the elevator to a specific position using closed-loop control.
     * The command ends when the elevator is near the target.
     * @param position Target position to move to
     * @return Command instance
     */
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

    /**
     * Creates a Command that directly applies a voltage to the elevator motors.
     * Used for fine/manual control.
     * @param voltage Voltage to apply
     * @return Command instance
     */
    private Command moveVoltage(Voltage voltage) {
        return startEnd(
            () -> {
                VoltageOut controlRequest = new VoltageOut(voltage);

                leftMotor.setControl(controlRequest);
                rightMotor.setControl(controlRequest);
                this.controlRequest = controlRequest;
            },
            () -> startPositionControl(getPosition(), true)
        );
    }

    /**
     * @return Command for fine upward manual control (low positive voltage)
     */
    public Command fineControlUp() {
        return moveVoltage(ElevatorConstants.FINE_CONTROL_UP);
    }

    /**
     * @return Command for fine downward manual control (low negative voltage)
     */
    public Command fineControlDown() {
        return moveVoltage(ElevatorConstants.FINE_CONTROL_DOWN);
    }

    /**
     * Main periodic method.
     * Handles zeroing on the bottom limit switch, disables elevator when robot is disabled,
     * and ensures safety (reapplies limits and control requests periodically).
     */
    @Override
    public void periodic() {
        refreshStatusSignals();

        // Zero elevator if at bottom and not yet zeroed
        if (bottomLimitSwitchTriggered() && !elevatorZeroed) {
            CTREUtils.check(leftMotor.setPosition(ElevatorConstants.MIN_HEIGHT.in(Meters)));
            CTREUtils.check(rightMotor.setPosition(ElevatorConstants.MIN_HEIGHT.in(Meters)));

            elevatorZeroed = true;
            startPositionControl(ElevatorConstants.MIN_HEIGHT, true);
            return;
        }

        // Hold current position when robot is disabled
        if (RobotState.isDisabled()) {
            startPositionControl(getPosition(), true);
        }

        // Periodically reapply limits and control requests for safety
        if (controlRequest != null) {
            controlRequest = setLimits(controlRequest);
            CTREUtils.check(leftMotor.setControl(controlRequest));
            CTREUtils.check(rightMotor.setControl(controlRequest));
        }
    }

    /**
     * Sets limit switches for the current control request, to prevent overtravel.
     * @param request The control request
     * @return The same request with updated limits
     */
    private ControlRequest setLimits(ControlRequest request) {
        return setLimits(request, topLimitSwitchTriggered() || !elevatorZeroed, bottomLimitSwitchTriggered());
    }

    /**
     * Utility to set fwd/rev motion limits for various control request types.
     * @param request Control request object (MotionMagicVoltage, VoltageOut, VelocityVoltage)
     * @param forward True to enable forward (up) limit
     * @param reverse True to enable reverse (down) limit
     * @return Modified request
     */
    private static ControlRequest setLimits(ControlRequest request, boolean forward, boolean reverse) {
        if (request instanceof MotionMagicVoltage r) {
            return r.withLimitForwardMotion(forward).withLimitReverseMotion(reverse);
        } else if (request instanceof VoltageOut r) {
            return r.withLimitForwardMotion(forward).withLimitReverseMotion(reverse);
        } else if (request instanceof VelocityVoltage r) {
            return r.withLimitForwardMotion(forward).withLimitReverseMotion(reverse);
        } else {
            return request;
        }
    }

    // -------------------------------------------------------------------------
    // Predefined Preset Positions / Commands
    // -------------------------------------------------------------------------

    /** @return Command to stow elevator (lowest safe position) */
    public Command stow() {
        return moveToPosition(Constants.ELEVATOR.STOW_HEIGHT);
    }

    /** @return Command to move to Coral Level 1 scoring height */
    public Command coralL1() {
        return moveToPosition(Constants.ELEVATOR.CORAL.L1_HEIGHT);
    }

    /** @return Command to move to Coral Level 2 scoring height */
    public Command coralL2() {
        return moveToPosition(Constants.ELEVATOR.CORAL.L2_HEIGHT);
    }

    /** @return Command to move to Coral Level 3 scoring height */
    public Command coralL3() {
        return moveToPosition(Constants.ELEVATOR.CORAL.L3_HEIGHT);
    }

    /** @return Command to move to Coral Level 4 scoring height */
    public Command coralL4() {
        return moveToPosition(Constants.ELEVATOR.CORAL.L4_HEIGHT);
    }

    /** @return Command to move to Coral intake position */
    public Command coralIntake() {
        return moveToPosition(Constants.ELEVATOR.CORAL.INTAKE_HEIGHT);
    }

    /** @return Command to move to Algae Level 2 scoring height */
    public Command algaeL2() {
        return moveToPosition(Constants.ELEVATOR.ALGAE.L2_HEIGHT);
    }

    /** @return Command to move to Algae Level 3 scoring height */
    public Command algaeL3() {
        return moveToPosition(Constants.ELEVATOR.ALGAE.L3_HEIGHT);
    }

    /** @return Command to move to Algae barge position */
    public Command algaeBarge() {
        return moveToPosition(Constants.ELEVATOR.ALGAE.BARGE_HEIGHT);
    }

    /** @return Command for auto "ready" height (for pre-load or pre-scoring) */
    public Command ready() {
        return moveToPosition(Constants.ELEVATOR.AUTO.READY_HEIGHT);
    }

    /**
     * Command for launching mechanism: rapidly moves elevator down, then up, then slows.
     * @return Composite command for launch barge sequence
     */
    public Command launchBarge() {
        return moveToPosition(Constants.ELEVATOR.MIN_HEIGHT)
            .andThen(moveVoltage(Volts.of(12)).until(() -> getPosition().gte(Inches.of(62.5))))
            .andThen(moveVoltage(Volts.of(-6)).until(() -> getVelocity().lte(InchesPerSecond.of(2))));
    }

    /**
     * Command for moving elevator to a specified Coral level.
     * @param level Level (1-4)
     * @return Command for that level
     */
    public Command coral(int level) {
        return switch (level) {
            case 1 -> coralL1();
            case 2 -> coralL2();
            case 3 -> coralL3();
            case 4 -> coralL4();
            default -> throw new IllegalArgumentException("Invalid coral level: " + level);
        };
    }

    /**
     * Command for moving elevator to a specified Algae level.
     * @param level Level (2-3)
     * @return Command for that level
     */
    public Command algae(int level) {
        return switch (level) {
            case 2 -> algaeL2();
            case 3 -> algaeL3();
            default -> throw new IllegalArgumentException("Invalid algae level: " + level);
        };
    }

    /**
     * Factory method for creating an Elevator instance.
     * Returns a simulated elevator in simulation, or a real elevator on hardware.
     * @param grabber The Grabber subsystem
     * @return Elevator or SimElevator instance
     */
    public static Elevator create(Grabber grabber) {
        return RobotBase.isReal() ? new Elevator(grabber) : new SimElevator(grabber);
    }
}
