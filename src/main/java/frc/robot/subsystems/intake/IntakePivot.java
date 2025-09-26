package frc.robot.subsystems.intake;

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

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivot extends SubsystemBase {
    private TalonFX motor;
    private CANcoder encoder;

    private StatusSignal<Angle> positionIn;
    private StatusSignal<AngularVelocity> velocityIn;
    private StatusSignal<Current> statorCurrentIn;
    private StatusSignal<Current> supplyCurrentIn;

    private ControlRequest controlRequest;

    public IntakePivot() {
        motor = new TalonFX(IntakeConstants.pivotMotorID, IntakeConstants.canBus);
        encoder = new CANcoder(IntakeConstants.absoluteEncoderID, IntakeConstants.canBus);

        TalonFXConfiguration motorConfig = IntakeConstants.pivotMotorConfiguration;

        motorConfig.Feedback
            .withRotorToSensorRatio(IntakeConstants.pivotRotorToSensor)
            .withSensorToMechanismRatio(IntakeConstants.pivotSensorToMechanism)
            .withRemoteCANcoder(encoder);

        CTREUtils.check(motor.getConfigurator().apply(motorConfig));

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

        encoderConfig.MagnetSensor
            .withSensorDirection(IntakeConstants.absoluteEncoderDirection)
            .withMagnetOffset(IntakeConstants.absoluteEncoderOffset.plus(IntakeConstants.centerOfMassAngularOffset));
        
        CTREUtils.check(encoder.getConfigurator().apply(encoderConfig));

        Logger.logMeasure("Intake/position", this::getPosition);
        Logger.logMeasure("Intake/velocity", this::getVelocity);

        initStatusSignals();
    }

    private void initStatusSignals() {
        positionIn = motor.getPosition();
        velocityIn = motor.getVelocity();
        statorCurrentIn = motor.getStatorCurrent();
        supplyCurrentIn = motor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            Hertz.of(100),
            positionIn, velocityIn,
            encoder.getPosition(), encoder.getVelocity() // TODO: Test to see if this is needed
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
            Hertz.of(50),
            statorCurrentIn, supplyCurrentIn
        );

        ParentDevice.optimizeBusUtilizationForAll(motor, encoder);
    }

    private void refreshStatusSignals() {
        CTREUtils.check(positionIn.getStatus());
        CTREUtils.check(velocityIn.getStatus());
        CTREUtils.check(statorCurrentIn.getStatus());
        CTREUtils.check(supplyCurrentIn.getStatus());

        BaseStatusSignal.refreshAll(
            positionIn, velocityIn, statorCurrentIn, supplyCurrentIn
        );
    }

    public Angle getPosition() {
        return CTREUtils.unwrap(positionIn).minus(IntakeConstants.centerOfMassAngularOffset);
    }

    public AngularVelocity getVelocity() {
        return CTREUtils.unwrap(velocityIn);
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

    private void startPivotingTo(Angle position) {
        controlRequest = new PositionVoltage(position.plus(IntakeConstants.centerOfMassAngularOffset));
        applyLimitsToControlRequest(controlRequest);
        CTREUtils.check(motor.setControl(controlRequest));
    }

    public Command holdAt(Angle position) {
        return startEnd(() -> startPivotingTo(position), () -> startPivotingTo(getPosition()));
    }

    public Command pivotTo(Angle position) {
        return holdAt(position).until(() -> isNear(position));
    }

    public boolean isNear(Angle position) {
        return getPosition().isNear(position, IntakeConstants.pivotTolerance);
    }

    public Command holdAtStow() {
        return holdAt(IntakeConstants.pivotStowAngle);
    }

    public Command pivotToStow() {
        return pivotTo(IntakeConstants.pivotStowAngle);
    }

    public Command holdAtIntake() {
        return holdAt(IntakeConstants.pivotIntakeAngle);
    }

    public Command pivotToIntake() {
        return pivotTo(IntakeConstants.pivotIntakeAngle);
    }

    @Override
    public void periodic() {
        refreshStatusSignals();

        applyLimitsToControlRequest(controlRequest);
        CTREUtils.check(motor.setControl(controlRequest));
    }
}
