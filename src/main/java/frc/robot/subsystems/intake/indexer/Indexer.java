package frc.robot.subsystems.intake.indexer;

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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants;

public class Indexer extends SubsystemBase {
    protected TalonFX motor;

    private StatusSignal<AngularVelocity> velocityIn;
    private StatusSignal<Current> statorCurrentIn;
    private StatusSignal<Current> supplyCurrentIn;
    private StatusSignal<Voltage> appliedVoltageIn;
    
    private Voltage simVoltage;

    private boolean isIntaking = false;
    private boolean isDropping = false;

    public Indexer() {
        motor = new TalonFX(IntakeConstants.indexerMotorId, IntakeConstants.canBus);

        motor.getConfigurator().apply(IntakeConstants.indexerMotorConfiguration);

        initStatusSignals();

        if (RobotBase.isReal()) {
            Logger.logMeasure("Intake/Indexer/velocity", () -> CTREUtils.unwrap(velocityIn));
            Logger.logMeasure("Intake/Indexer/statorCurrent", () -> CTREUtils.unwrap(statorCurrentIn));
            Logger.logMeasure("Intake/Indexer/supplyCurrent", () -> CTREUtils.unwrap(supplyCurrentIn));
            Logger.logMeasure("Intake/Indexer/appliedVoltage", () -> CTREUtils.unwrap(appliedVoltageIn));
        } else {
            Logger.logMeasure("Intake/Indexer/appliedVoltage", () -> simVoltage);
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
     * Runs the indexer motor to move a piece of coral into the manipulator. The
     * returned command does not end on its own; it must be interrupted or
     * canceled.
     * 
     * @return A command that runs the indexer motor to intake coral.
     */
    public Command intake() {
        return run(IntakeConstants.indexerIntakeVoltage).alongWith(Commands.startEnd(
            () -> isIntaking = true,
            () -> isIntaking = false
        ));
    }

    /**
     * Runs the indexer motor in reverse to drop a piece of coral out of the
     * robot. The returned command does not end on its own; it must be
     * interrupted or canceled.
     * 
     * @return A command that runs the indexer motor to drop coral.
     */
    public Command drop() {
        return run(IntakeConstants.indexerDropVoltage).alongWith(Commands.startEnd(
            () -> isDropping = true,
            () -> isDropping = false
        ));
    }

    public boolean isDropping() {
        return isDropping;
    }

    public boolean isIntaking() {
        return isIntaking;
    }

    @Override
    public void periodic() {
        refreshStatusSignals();

        if (RobotState.isDisabled()) setVoltage(Volts.of(0));
    }
}
