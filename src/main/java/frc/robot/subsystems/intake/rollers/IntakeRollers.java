package frc.robot.subsystems.intake.rollers;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.utils.CTREUtils;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants;

/**
 * The intake rollers subsystem, which pulls coral off of the ground and into
 * the robot.
 * <h3>Controlling the Rollers</h3>
 * Generally, the rollers should be only controlled via the {@link #intake()}
 * method, which runs the rollers in the direction and speed that intakes coral
 * from the ground. This method returns a command that can be scheduled or added
 * to a composition to run the rollers.
 * <h3>Simulation</h3>
 * In simulation, instantiate an instance of the {@link IntakeRollersSim}
 * subclass instead of this one. The IntakeRollersSim class fully simulates the
 * rollers' physics and motor behavior.
 */
public class IntakeRollers extends SubsystemBase {
    protected TalonFX motor;

    private StatusSignal<AngularVelocity> velocityIn;
    private StatusSignal<Current> statorCurrentIn;
    private StatusSignal<Current> supplyCurrentIn;
    private StatusSignal<Voltage> appliedVoltageIn;

    /**
     * Creates a new IntakeRollers subsystem.
     */
    public IntakeRollers() {
        motor = new TalonFX(IntakeConstants.rollerMotorID, IntakeConstants.canBus);

        motor.getConfigurator().apply(IntakeConstants.rollerMotorConfiguration);

        initStatusSignals();

        Logger.logMeasure("GroundCoralRollers/velocity", () -> CTREUtils.unwrap(velocityIn));
        Logger.logMeasure("GroundCoralRollers/statorCurrent", () -> CTREUtils.unwrap(statorCurrentIn));
        Logger.logMeasure("GroundCoralRollers/supplyCurrent", () -> CTREUtils.unwrap(supplyCurrentIn));
        Logger.logMeasure("GroundCoralRollers/appliedVoltage", () -> CTREUtils.unwrap(appliedVoltageIn));
    }

    private void initStatusSignals() {
        velocityIn = motor.getVelocity();
        statorCurrentIn = motor.getStatorCurrent();
        supplyCurrentIn = motor.getSupplyCurrent();
        appliedVoltageIn = motor.getMotorVoltage();

        CTREUtils.check(BaseStatusSignal.setUpdateFrequencyForAll(
            Hertz.of(50),
            velocityIn, statorCurrentIn, supplyCurrentIn, appliedVoltageIn
        ));

        CTREUtils.check(motor.optimizeBusUtilization());
    }

    private void refreshStatusSignals() {
        CTREUtils.check(BaseStatusSignal.refreshAll(
            velocityIn, statorCurrentIn, supplyCurrentIn, appliedVoltageIn
        ));
    }

    private void setVoltage(Voltage voltage) {
        motor.setVoltage(voltage.in(Volts));
    }

    private Command run(Voltage voltage) {
        return startEnd(
            () -> setVoltage(voltage),
            () -> setVoltage(Volts.of(0))
        );
    }

    /**
     * Runs the intake rollers in a way that intakes coral from the ground.
     * @return a command that runs the rollers to intake coral
     */
    public Command intake() {
        return run(IntakeConstants.rollerIntakeVoltage);
    }

    @Override
    public void periodic() {
        refreshStatusSignals();

        if (RobotState.isDisabled()) setVoltage(Volts.of(0));
    }
}
