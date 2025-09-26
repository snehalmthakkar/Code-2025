package frc.robot.subsystems.intake.indexer;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.team6962.lib.telemetry.Logger;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants;

public class Indexer extends SubsystemBase {
    private TalonFX motor;

    private double appliedVoltageVolts;

    public Indexer() {
        motor = new TalonFX(IntakeConstants.indexerMotorId, IntakeConstants.canBus);

        motor.getConfigurator().apply(IntakeConstants.indexerMotorConfiguration);

        Logger.logNumber("Intake/indexerVoltageVolts", () -> appliedVoltageVolts);
    }

    private void setVoltage(Voltage voltage) {
        appliedVoltageVolts = voltage.in(Volts);
        motor.setVoltage(voltage.in(Volts));
    }

    /**
     * Runs the indexer motor to move a piece of coral into the manipulator. The
     * returned command does not end on its own; it must be interrupted or
     * canceled.
     * 
     * @return A command that runs the indexer motor to intake coral.
     */
    public Command intake() {
        return startEnd(() -> setVoltage(IntakeConstants.indexerIntakeVoltage), () -> setVoltage(Volts.of(0)));
    }

    /**
     * Runs the indexer motor in reverse to drop a piece of coral out of the
     * robot. The returned command does not end on its own; it must be
     * interrupted or canceled.
     * 
     * @return A command that runs the indexer motor to drop coral.
     */
    public Command drop() {
        return startEnd(() -> setVoltage(IntakeConstants.indexerDropVoltage), () -> setVoltage(Volts.of(0)));
    }

    public boolean isDropping() {
        return (Math.signum(appliedVoltageVolts) == Math.signum(IntakeConstants.indexerDropVoltage.in(Volts))) && Math.abs(appliedVoltageVolts) > 1;
    }

    public boolean isIntaking() {
        return (Math.signum(appliedVoltageVolts) == Math.signum(IntakeConstants.indexerIntakeVoltage.in(Volts))) && Math.abs(appliedVoltageVolts) > 1;
    }
}
