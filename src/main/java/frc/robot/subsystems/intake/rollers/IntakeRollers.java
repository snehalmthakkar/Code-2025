package frc.robot.subsystems.intake.rollers;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.utils.CTREUtils;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants;

/**
 * The intake rollers subsystem, which pulls coral off of the ground and into
 * the robot.
 * <h3>Controlling the Rollers</h3>
 * The rollers can be moved via the {@link #intake()} method, which runs the
 * rollers in the direction and speed that intakes coral from the ground. The
 * intake() method returns a command that can be scheduled or added to a
 * composition to run the rollers.
 */
public class IntakeRollers extends SubsystemBase {
    protected TalonFX motor;

    private StatusSignal<AngularVelocity> velocityIn;
    private StatusSignal<Current> statorCurrentIn;
    private StatusSignal<Current> supplyCurrentIn;
    private StatusSignal<Voltage> appliedVoltageIn;
    
    private Voltage simVoltage;

    /**
     * Creates a new IntakeRollers subsystem.
     */
    public IntakeRollers() {
        motor = new TalonFX(IntakeConstants.rollerMotorID, IntakeConstants.canBus);

        motor.getConfigurator().apply(IntakeConstants.rollerMotorConfiguration);

        initStatusSignals();

        if (RobotBase.isReal()) {
            Logger.logMeasure("Intake/Rollers/velocity", () -> CTREUtils.unwrap(velocityIn));
            Logger.logMeasure("Intake/Rollers/statorCurrent", () -> CTREUtils.unwrap(statorCurrentIn));
            Logger.logMeasure("Intake/Rollers/supplyCurrent", () -> CTREUtils.unwrap(supplyCurrentIn));
            Logger.logMeasure("Intake/Rollers/appliedVoltage", () -> CTREUtils.unwrap(appliedVoltageIn));
        } else {
            Logger.logMeasure("Intake/Rollers/appliedVoltage", () -> simVoltage);
        }
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
        if (RobotBase.isSimulation()) {
            simVoltage = voltage;
        }

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

    public Command drop() {
        return run(IntakeConstants.rollerDropVoltage);
    }

    /**
     * Returns whether the intake is currently moving. This is determined by
     * checking if the rollers are spinning at a non-negligible speed.
     * 
     * @return whether the intake is currently intaking coral
     */
    public boolean isIntaking() {
        return RobotBase.isSimulation() ? Math.abs(simVoltage.in(Volts)) > 1 : Math.abs(CTREUtils.unwrap(velocityIn).in(RotationsPerSecond)) > 1;
    }

    @Override
    public void periodic() {
        refreshStatusSignals();

        if (RobotState.isDisabled()) setVoltage(Volts.of(0));
    }
}
