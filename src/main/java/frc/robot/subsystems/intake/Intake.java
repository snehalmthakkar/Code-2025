package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.IntakeSensors.CoralLocation;
import frc.robot.subsystems.intake.indexer.Indexer;
import frc.robot.subsystems.intake.pivot.IntakePivot;
import frc.robot.subsystems.intake.pivot.IntakePivotSim;
import frc.robot.subsystems.intake.rollers.IntakeRollers;
import frc.robot.subsystems.manipulator.grabber.Grabber;

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
 * The {@link #coralInIntake()} method can be used to check whether the intake
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
    public final Indexer indexer;
    public final IntakeSensors sensors;

    /**
     * Creates a new Intake subsystem.
     */
    public Intake(Grabber grabber) {
        indexer = new Indexer();
        rollers = new IntakeRollers();
        sensors = new IntakeSensors(grabber, indexer);

        if (RobotBase.isSimulation()) {
            pivot = new IntakePivotSim(sensors);
        } else {
            pivot = new IntakePivot(sensors);
        }
    }

    /**
     * Returns a command that intakes a piece of coral from the ground. This
     * command will lower the pivot and run the rollers until a piece of coral
     * has been detected to have completely passed through the intake, and it is
     * safe to raise the intake again.
     * @return the command that intakes a piece of coral from the ground
     */
    public Command intake() {
        Command command = Commands.parallel(
            rollers.intake().until(() -> sensors.getCoralLocation() == CoralLocation.INDEXER),
            pivot.deploy().until(() -> sensors.getCoralLocation() == CoralLocation.INTAKE || sensors.getCoralLocation() == CoralLocation.INDEXER),
            indexer.intake().until(() -> sensors.getCoralLocation() == CoralLocation.INDEXER)
        );

        if (RobotBase.isSimulation()) {
            command = command.deadlineFor(Commands.sequence(
                Commands.waitSeconds(0.25),
                Commands.runOnce(() -> sensors.simIntakeCoral()),
                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> sensors.simIndexCoral())
            ));
        }

        return command;
    }

    /**
     * Returns a command that drops a piece of coral from the indexer onto the
     * ground. This command will stow the pivot, run the indexer in
     * reverse until the coral has exited the robot, and then run the indexer
     * for a short time to ensure the coral is fully clear of the robot.
     * @return the command that drops a piece of coral from the indexer
     */
    public Command drop() {
        Command command = Commands.sequence(
            pivot.stow(),
            indexer.drop().until(() -> sensors.getCoralLocation() == CoralLocation.OUTSIDE),
            Commands.waitTime(IntakeConstants.indexerDropExtraTime)
        );

        if (RobotBase.isSimulation()) {
            command = command.deadlineFor(Commands.sequence(
                Commands.waitSeconds(0.25),
                Commands.runOnce(() -> sensors.simDropCoral())
            ));
        }

        return command;
    }

    /**
     * Returns a command that stows the intake by raising the pivot to its
     * stowed position, where it won't hit anything.
     * @return the command that stows the intake
     */
    public Command stow() {
        return pivot.stow();
    }

    public Command transfer() {
        Command command = indexer.intake()
            .until(() -> sensors.getCoralLocation() == CoralLocation.OUTSIDE);

        if (RobotBase.isSimulation()) {
            command = command.deadlineFor(Commands.sequence(
                Commands.waitSeconds(0.25),
                Commands.runOnce(() -> sensors.simTransferCoral())
            ));
        }

        return command;
    }

    public CoralLocation getCoralLocation() {
        return sensors.getCoralLocation();
    }
}
