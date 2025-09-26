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

public class GroundIntakePivot extends SubsystemBase {
    private TalonFX motor;
    private CANcoder encoder;

    private StatusSignal<Angle> positionIn;
    private StatusSignal<AngularVelocity> velocityIn;
    private StatusSignal<Current> statorCurrentIn;
    private StatusSignal<Current> supplyCurrentIn;

    private ControlRequest controlRequest;

    public GroundIntakePivot() {
        motor = new TalonFX(GroundIntakeConstants.motorID, GroundIntakeConstants.canBus);
        encoder = new CANcoder(GroundIntakeConstants.absoluteEncoderID, GroundIntakeConstants.canBus);

        TalonFXConfiguration motorConfig = GroundIntakeConstants.motorConfiguration;

        motorConfig.Feedback
            .withRotorToSensorRatio(GroundIntakeConstants.gearReduction)
            .withRemoteCANcoder(encoder);

        CTREUtils.check(motor.getConfigurator().apply(motorConfig));

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

        encoderConfig.MagnetSensor
            .withSensorDirection(GroundIntakeConstants.absoluteEncoderDirection)
            .withMagnetOffset(GroundIntakeConstants.absoluteEncoderOffset.plus(GroundIntakeConstants.centerOfMassAngularOffset));
        
        CTREUtils.check(encoder.getConfigurator().apply(encoderConfig));

        // Logger.log("GroundCoralIntake/pivotAngle", this::getPosition);

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
        return CTREUtils.unwrap(positionIn).minus(GroundIntakeConstants.centerOfMassAngularOffset);
    }

    public AngularVelocity getVelcoity() {
        return CTREUtils.unwrap(velocityIn);
    }

    private boolean limitReverseMotion() {
        return getPosition().in(Degrees) <= GroundIntakeConstants.minAngle.in(Degrees);
    }

    private boolean limitForwardMotion() {
        return getPosition().in(Degrees) >= GroundIntakeConstants.maxAngle.in(Degrees);
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

    public Command 

    @Override
    public void periodic() {
        refreshStatusSignals();

        applyLimitsToControlRequest(controlRequest);
        CTREUtils.check(motor.setControl(controlRequest));
    }
}
