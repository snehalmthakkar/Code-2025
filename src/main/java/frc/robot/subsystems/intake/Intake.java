package frc.robot.subsystems.intake;

import com.team6962.lib.telemetry.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.pivot.IntakePivot;
import frc.robot.subsystems.intake.pivot.IntakePivotSim;
import frc.robot.subsystems.intake.rollers.IntakeRollers;
import frc.robot.subsystems.intake.sensor.IntakeSensor;

/**
 * The ground coral intake subsystem, which pulls coral off of the ground and
 * into the robot. This subsystem consists of two controllable subsystems: the
 * intake rollers, which spin to pull coral in, and the intake pivot, which
 * pivots the rollers down to the ground to intake coral and up to stow the
 * intake where it won't hit anything.
 * <h3>Controlling the Intake</h3>
 * Generally, the intake should be only controlled via the {@link #intake()} and
 * {@link #stow()} methods, which return {@link Command Commands}. The
 * intake() command will lower the pivot and run the rollers until a piece of
 * coral has been detected to have completely passed through the intake. The
 * stow() command will raise the pivot to its stowed position, where it won't
 * hit anything.
 * <h3>Getting the Intake's State</h3>
 * The {@link #coralDetectedInEntry()} method can be used to check whether the intake
 * currently has a piece of coral in it. This is primarily useful in autonomous,
 * where the autonomous code may want to wait until the intake's beam break
 * sensor has detected a piece of coral before driving to score the coral.
 * <h3>Simulation</h3>
 * This class is compatible with both real and simulated hardware. In
 * simulation, the Intake will automatically switch to using simulated versions
 * of the rollers, pivot, and sensor subsystems that fully simulate the physics
 * and device behavior of those subsystems.
 */
public class Intake {
    public final IntakeRollers rollers;
    public final IntakePivot pivot;
    public final IntakeSensor entrySensor;
    public final IntakeSensor transferSensor;

    /**
     * Creates a new Intake subsystem.
     */
    public Intake() {
        if (RobotBase.isSimulation()) {
            pivot = new IntakePivotSim();
        } else {
            pivot = new IntakePivot();
        }

        rollers = new IntakeRollers();

        entrySensor = new IntakeSensor(IntakeConstants.entrySensorChannel, IntakeConstants.entrySensorWiring);
        transferSensor = new IntakeSensor(IntakeConstants.transferSensorChannel, IntakeConstants.transferSensorWiring);

        Logger.logDigitalSensor("Intake/coralDetectedInEntry", entrySensor);
        Logger.logDigitalSensor("Intake/coralDetectedInTransfer", transferSensor);
    }

    /**
     * Returns a command that intakes a piece of coral from the ground. This
     * command will lower the pivot and run the rollers until a piece of coral
     * has been detected to have completely passed through the intake, and it is
     * safe to raise the intake again.
     * @return the command that intakes a piece of coral from the ground
     */
    public Command intake() {
        Command intakeCommand = Commands.parallel(
            rollers.intake(),
            pivot.deploy()
        ).withDeadline(Commands.sequence(
            Commands.waitUntil(this::coralDetectedInEntry),
            Commands.waitUntil(this::coralDetectedInTransfer),
            Commands.waitUntil(() -> !coralDetectedInTransfer()),
            Commands.waitTime(IntakeConstants.delayAfterIntake)
        ));

        if (RobotBase.isSimulation()) {
            intakeCommand = intakeCommand.deadlineFor(Commands.waitSeconds(0.25).andThen(entrySensor::simulateDetection));
        }

        return intakeCommand;
    }

    /**
     * Returns a command that stows the intake by raising the pivot to its
     * stowed position, where it won't hit anything.
     * @return the command that stows the intake
     */
    public Command stow() {
        return pivot.stow();
    }

    /**
     * Returns whether the intake's beam break sensor is detecting that a piece
     * of coral is currently in the intake. This is primarily useful in
     * autonomous, where the autonomous code may want to wait until the intake's
     * beam break sensor has detected a piece of coral before driving to score
     * the coral.
     * @return whether the intake's beam break sensor is detecting coral
     */
    public boolean coralDetectedInEntry() {
        return entrySensor.isTriggered();
    }

    public boolean coralDetectedInTransfer() {
        return transferSensor.isTriggered();
    }
}
