package frc.robot.subsystems.intake.pivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.utils.CTREUtils;
import com.team6962.lib.utils.MeasureMath;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants;

/**
 * The intake pivot subsystem, which pivots the ground coral intake up and down.
 * <h3>Controlling the Pivot</h3>
 * The pivot can be controlled with the {@link #deploy()} method and
 * {@link #stow()} method, which return {@link Command Commands}.
 * <h3>Getting the Pivot's State</h3>
 * The position and velocity of the pivot can be read with the
 * {@link #getPosition()} and {@link #getVelocity()} methods.
 * <h3>Automatic Holding Behavior</h3>
 * When no command is actively controlling the pivot with an ongoing
 * {@link #moveTo()} or {@link #holdAt()} command, the pivot will use feedback
 * control to stay in its current position.
 * <h3>Simulation</h3>
 * In simulation, instantiate an instance of the {@link IntakePivotSim} subclass
 * instead of this one. The IntakePivotSim class fully simulates the pivot's
 * physics and motor behavior.
 * <h3>Internal Position Offset</h3>
 * The TalonFX's position is offset by the angle between the center of mass and
 * the external code's zero angle. This means that if the center of mass is
 * 10 degrees above the zero angle, then when the external code sets the pivot to
 * 0 degrees, the TalonFX will actually be set to 10 degrees. This offset is
 * handled internally and does not affect the external code's view of the
 * pivot's position.
 */
public class IntakePivot extends SubsystemBase {
    protected TalonFX motor;
    protected CANcoder encoder;

    private StatusSignal<Angle> positionIn;
    private StatusSignal<AngularVelocity> velocityIn;
    private StatusSignal<Current> statorCurrentIn;
    private StatusSignal<Current> supplyCurrentIn;
    private StatusSignal<Voltage> appliedVoltageIn;

    private ControlRequest controlRequest;

    /**
     * Creates a new IntakePivot.
     */
    public IntakePivot() {
        motor = new TalonFX(IntakeConstants.pivotMotorID, IntakeConstants.canBus);
        encoder = new CANcoder(IntakeConstants.absoluteEncoderID, IntakeConstants.canBus);

        TalonFXConfiguration motorConfig = IntakeConstants.pivotMotorConfiguration;

        motorConfig.Feedback
            .withRotorToSensorRatio(IntakeConstants.pivotSensorToMechanism)
            .withSensorToMechanismRatio(IntakeConstants.pivotSensorToMechanism)
            .withRemoteCANcoder(encoder);

        CTREUtils.check(motor.getConfigurator().apply(motorConfig));

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

        encoderConfig.MagnetSensor
            .withSensorDirection(IntakeConstants.absoluteEncoderDirection)
            .withMagnetOffset(IntakeConstants.absoluteEncoderOffset.plus(IntakeConstants.centerOfMassAngularOffset));
        
        CTREUtils.check(encoder.getConfigurator().apply(encoderConfig));

        initStatusSignals();

        Logger.logMeasure("IntakePivot/position", this::getPosition);
        Logger.logMeasure("IntakePivot/velocity", this::getVelocity);
        Logger.logMeasure("IntakePivot/statorCurrent", () -> CTREUtils.unwrap(statorCurrentIn));
        Logger.logMeasure("IntakePivot/supplyCurrent", () -> CTREUtils.unwrap(supplyCurrentIn));
        Logger.logMeasure("IntakePivot/appliedVoltage", () -> CTREUtils.unwrap(appliedVoltageIn));
    }

    /**
     * Gets the current position of the pivot.
     * @return The current position of the pivot, in a measure.
     */
    public Angle getPosition() {
        return CTREUtils.unwrap(positionIn).minus(IntakeConstants.centerOfMassAngularOffset);
    }

    /**
     * Gets the current velocity of the pivot.
     * @return The current velocity of the pivot, in a measure.
     */
    public AngularVelocity getVelocity() {
        return CTREUtils.unwrap(velocityIn);
    }

    /**
     * Creates a command that moves the pivot to the specified angle, ending
     * when the target is reached.
     * @param targetAngle The angle to move the pivot to.
     * @return A command that moves the pivot to the specified angle.
     */
    private Command moveTo(Angle targetAngle) {
        return startEnd(
            () -> setPositionControl(targetAngle),
            () -> {
                if (!isNear(targetAngle)) {
                    setPositionControl(getPosition());
                }
            }
        ).until(() -> MeasureMath.minAbsDifference(targetAngle, getPosition()).lt(IntakeConstants.pivotTolerance));
    }

    /**
     * Creates a command that moves the pivot to the intake position, ending
     * when the target is reached.
     * @return A command that moves the pivot to the intake position.
     */
    public Command deploy() {
        return moveTo(IntakeConstants.pivotIntakeAngle);
    }

    /**
     * Creates a command that moves the pivot to the stow position, ending
     * when the target is reached.
     * @return A command that moves the pivot to the stow position.
     */
    public Command stow() {
        return moveTo(IntakeConstants.pivotStowAngle);
    }

    /**
     * Returns whether the pivot is near the specified position, within the
     * tolerance specified in {@link IntakeConstants#pivotTolerance}.
     * 
     * @param position The position to check if the pivot is near to.
     * @return Whether the pivot is near the specified position.
     */
    public boolean isNear(Angle position) {
        return getPosition().isNear(position, IntakeConstants.pivotTolerance);
    }

    @Override
    public void periodic() {
        refreshStatusSignals();

        if (RobotState.isDisabled()) setPositionControl(getPosition());

        setControlWithLimits(controlRequest);
    }

    private void setPositionControl(Angle targetAngle) {
        setControlWithLimits(new PositionVoltage(targetAngle.plus(IntakeConstants.centerOfMassAngularOffset)));
    }

    private void setControlWithLimits(ControlRequest request) {
        controlRequest = request;

        applyLimitsToControlRequest(controlRequest);
        CTREUtils.check(motor.setControl(controlRequest));
    }

    private void initStatusSignals() {
        positionIn = motor.getPosition();
        velocityIn = motor.getVelocity();
        statorCurrentIn = motor.getStatorCurrent();
        supplyCurrentIn = motor.getSupplyCurrent();
        appliedVoltageIn = motor.getMotorVoltage();

        CTREUtils.check(BaseStatusSignal.setUpdateFrequencyForAll(
            Hertz.of(100),
            positionIn, velocityIn,
            encoder.getPosition(), encoder.getVelocity() // TODO: Test to see if this is needed
        ));

        CTREUtils.check(BaseStatusSignal.setUpdateFrequencyForAll(
            Hertz.of(50),
            statorCurrentIn, supplyCurrentIn, appliedVoltageIn
        ));

        CTREUtils.check(ParentDevice.optimizeBusUtilizationForAll(motor, encoder));
    }

    private void refreshStatusSignals() {
        CTREUtils.check(positionIn.getStatus());
        CTREUtils.check(velocityIn.getStatus());
        CTREUtils.check(statorCurrentIn.getStatus());
        CTREUtils.check(supplyCurrentIn.getStatus());
        CTREUtils.check(appliedVoltageIn.getStatus());

        CTREUtils.check(BaseStatusSignal.refreshAll(
            positionIn, velocityIn,
            statorCurrentIn, supplyCurrentIn, appliedVoltageIn
        ));
    }

    private boolean limitReverseMotion() {
        return getPosition().in(Degrees) <= IntakeConstants.minPivotAngle.in(Degrees);
    }

    private boolean limitForwardMotion() {
        return getPosition().in(Degrees) >= IntakeConstants.maxPivotAngle.in(Degrees);
    }
    
    private void applyLimitsToControlRequest(ControlRequest request) {
        if (request == null) return;

        if (request instanceof MotionMagicVoltage r) {
            r.withLimitForwardMotion(limitForwardMotion()).withLimitReverseMotion(limitReverseMotion());
        } else if (request instanceof MotionMagicExpoVoltage r) {
            r.withLimitForwardMotion(limitForwardMotion()).withLimitReverseMotion(limitReverseMotion());
        } else if (request instanceof PositionVoltage r) {
            r.withLimitForwardMotion(limitForwardMotion()).withLimitReverseMotion(limitReverseMotion());
        }
    }
}
