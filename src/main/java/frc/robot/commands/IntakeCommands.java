package frc.robot.commands;

// Import necessary WPILib and robot-specific classes
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants.ELEVATOR;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeSensors.CoralLocation;
import frc.robot.subsystems.manipulator.Manipulator;

/**
 * Utility class providing static factory methods for various intake-related command sequences.
 * These commands coordinate the Intake, Elevator, and Manipulator subsystems to perform
 * complex actions for coral (game piece) manipulation.
 */
public final class IntakeCommands {

    // Private constructor prevents instantiation of this utility class
    private IntakeCommands() {
    }

    /**
     * Creates a command sequence to intake a coral and transfer it,
     * coordinating the intake, elevator, manipulator, and safety systems.
     *
     * @param intake           The Intake subsystem
     * @param elevator         The Elevator subsystem
     * @param manipulator      The Manipulator subsystem
     * @param safeSubsystems   Safety-aware command utilities
     * @param pieceCombos      Command utilities for piece manipulation
     * @return A Command that performs the intake and transfer sequence
     */
    public static Command intakeTransfer(Intake intake, Elevator elevator, Manipulator manipulator, SafeSubsystems safeSubsystems, PieceCombos pieceCombos) {
        // Sequence of actions: intake, move, stow, transfer
        return Commands.sequence(
            // First, run intake sequence until coral is at indexer
            Commands.deadline(
                Commands.parallel(
                    // Start intake rollers
                    intake.rollers.intake(),
                    // Deploy intake pivot and wait until coral is detected at intake or indexer, then stow
                    intake.pivot.deploy().until(() -> intake.sensors.getCoralLocation() == CoralLocation.INTAKE || intake.sensors.getCoralLocation() == CoralLocation.INDEXER)
                        .andThen(
                            Commands.waitUntil(() -> intake.sensors.getCoralLocation() == CoralLocation.INDEXER),
                            intake.pivot.stow()
                        ),
                    // Run indexer intake
                    intake.indexer.intake()
                ).until(() -> intake.sensors.getCoralLocation() == CoralLocation.INDEXER),
                // Move elevator and manipulator to coral intake height, safely
                safeSubsystems.safeMoveCommand(elevator.coralIntake(), manipulator.pivot.coralIntake(), ELEVATOR.CORAL.INTAKE_HEIGHT)
            )
            // Only run if manipulator does not already have coral, and coral is at required locations
            .onlyIf(() -> !manipulator.grabber.hasCoral() && (intake.sensors.getCoralLocation() == CoralLocation.OUTSIDE || intake.sensors.getCoralLocation() == CoralLocation.INTAKE || intake.sensors.getCoralLocation() == CoralLocation.TRANSFER_TO_INDEXER)),
            
            // Second, stow pivot while holding elevator/manipulator at intake height
            Commands.deadline(
                safeSubsystems.safeMoveCommand(elevator.coralIntake(), manipulator.pivot.coralIntake(), ELEVATOR.CORAL.INTAKE_HEIGHT),
                intake.pivot.stow()
            ),
            // Third, transfer the coral to manipulator and invoke piece combo logic
            Commands.deadline(
                intake.transfer(),
                pieceCombos.intakeCoral(),
                intake.pivot.stow()
            )
            // Only perform transfer if coral is at indexer/transfer-to-manipulator, or grabber is not clear
            .onlyIf(() -> intake.sensors.getCoralLocation() == CoralLocation.INDEXER || intake.sensors.getCoralLocation() == CoralLocation.TRANSFER_TO_MANIPULATOR || !manipulator.grabber.isCoralClear())
        );
    }

    /**
     * Autonomous version of intake and transfer sequence.
     * This version simulates coral intake in simulation and otherwise
     * runs a reduced intake-transfer logic for autonomous routines.
     *
     * @param intake           The Intake subsystem
     * @param elevator         The Elevator subsystem
     * @param manipulator      The Manipulator subsystem
     * @param safeSubsystems   Safety-aware command utilities
     * @param pieceCombos      Command utilities for piece manipulation
     * @return A Command for autonomous intake and transfer
     */
    public static Command intakeTransferAuto(Intake intake, Elevator elevator, Manipulator manipulator, SafeSubsystems safeSubsystems, PieceCombos pieceCombos) {
        // In simulation, randomly simulate coral intake event
        if (RobotBase.isSimulation()) {
            return Commands.either(
                // Simulate successful coral intake after 2 seconds
                Commands.waitSeconds(2).andThen(() -> intake.sensors.simIntakeCoral()),
                // Otherwise, do nothing (wait forever)
                Commands.waitUntil(() -> false),
                // 75% chance of simulating intake
                () -> Math.random() < 0.75
            ).onlyIf(() -> intake.sensors.getCoralLocation() == CoralLocation.OUTSIDE)
                // Wait a bit after simulation
                .andThen(Commands.waitSeconds(0.5));
        }

        // On real robot, sequence similar to teleop transfer but streamlined for auto
        return Commands.sequence(
            // Intake sequence
            Commands.deadline(
                Commands.parallel(
                    intake.rollers.intake(),
                    intake.pivot.deploy().until(() -> intake.sensors.getCoralLocation() == CoralLocation.INTAKE || intake.sensors.getCoralLocation() == CoralLocation.INDEXER),
                    intake.indexer.intake()
                ).until(() -> intake.sensors.getCoralLocation() == CoralLocation.INDEXER),
                safeSubsystems.safeMoveCommand(elevator.coralIntake(), manipulator.pivot.coralIntake(), ELEVATOR.CORAL.INTAKE_HEIGHT)
            ).onlyIf(() -> !manipulator.grabber.hasCoral() && (intake.sensors.getCoralLocation() == CoralLocation.OUTSIDE || intake.sensors.getCoralLocation() == CoralLocation.INTAKE || intake.sensors.getCoralLocation() == CoralLocation.TRANSFER_TO_INDEXER)),
            // Hold at intake height
            safeSubsystems.safeMoveCommand(elevator.coralIntake(), manipulator.pivot.coralIntake(), ELEVATOR.CORAL.INTAKE_HEIGHT),
            // Transfer coral after indexing
            Commands.deadline(
                intake.transfer(),
                pieceCombos.intakeCoral()
            ).onlyIf(() -> intake.sensors.getCoralLocation() == CoralLocation.INDEXER || intake.sensors.getCoralLocation() == CoralLocation.TRANSFER_TO_MANIPULATOR || !manipulator.grabber.isCoralClear())
        );
    }

    /**
     * Command for vertical intake and transfer of coral.
     * This sequence is used when coral is to be picked up from a vertical orientation.
     *
     * @param intake           The Intake subsystem
     * @param elevator         The Elevator subsystem
     * @param manipulator      The Manipulator subsystem
     * @param safeSubsystems   Safety-aware command utilities
     * @param pieceCombos      Command utilities for piece manipulation
     * @return A Command for vertical coral intake and transfer
     */
    public static Command intakeTransferVertical(Intake intake, Elevator elevator, Manipulator manipulator, SafeSubsystems safeSubsystems, PieceCombos pieceCombos) {
        return Commands.sequence(
            // Run vertical intake sequence
            Commands.deadline(
                Commands.sequence(
                    // Deploy vertically and run rollers until coral detected
                    Commands.parallel(
                        intake.rollers.intake(),
                        intake.pivot.deployVertical()
                    ).until(() -> intake.sensors.getCoralLocation() == CoralLocation.INTAKE),
                    // Then deploy normally
                    intake.pivot.deploy(),
                    // Continue intake and indexer rollers until coral at indexer
                    Commands.parallel(
                        intake.rollers.intake(),
                        intake.indexer.intake()
                    ).until(() -> intake.sensors.getCoralLocation() == CoralLocation.INDEXER)
                ),
                // Move elevator and manipulator to intake height
                safeSubsystems.safeMoveCommand(elevator.coralIntake(), manipulator.pivot.coralIntake(), ELEVATOR.CORAL.INTAKE_HEIGHT)
            ).onlyIf(() -> !manipulator.grabber.hasCoral() && (intake.sensors.getCoralLocation() == CoralLocation.OUTSIDE || intake.sensors.getCoralLocation() == CoralLocation.TRANSFER_TO_INDEXER)),
            // Stow pivot after intake
            Commands.deadline(
                safeSubsystems.safeMoveCommand(elevator.coralIntake(), manipulator.pivot.coralIntake(), ELEVATOR.CORAL.INTAKE_HEIGHT),
                intake.pivot.stow()
            ),
            // Transfer coral to manipulator if available
            Commands.deadline(
                intake.transfer(),
                pieceCombos.intakeCoral(),
                intake.pivot.stow()
            ).onlyIf(() -> intake.sensors.getCoralLocation() == CoralLocation.INDEXER || intake.sensors.getCoralLocation() == CoralLocation.TRANSFER_TO_MANIPULATOR)
        );
    }

    /**
     * Provides a basic coral intake command with elevator and manipulator at correct height.
     * Can be called independently for just picking up coral.
     *
     * @param intake           The Intake subsystem
     * @param elevator         The Elevator subsystem
     * @param manipulator      The Manipulator subsystem
     * @param safeSubsystems   Safety-aware command utilities
     * @param pieceCombos      Command utilities for piece manipulation
     * @return A Command for coral intake only
     */
    public static Command intakeCoral(Intake intake, Elevator elevator, Manipulator manipulator, SafeSubsystems safeSubsystems, PieceCombos pieceCombos) {
        return Commands.deadline(
            // Run intake
            intake.intake(),
            // Move elevator/manipulator to correct height
            safeSubsystems.safeMoveCommand(elevator.coralIntake(), manipulator.pivot.coralIntake(), ELEVATOR.CORAL.INTAKE_HEIGHT)
        )
        // Only run if simulation, or manipulator doesn't already have coral and coral is outside
        .onlyIf(() -> RobotBase.isSimulation() || (!manipulator.grabber.hasCoral() && intake.sensors.getCoralLocation() == CoralLocation.OUTSIDE));
    }

    /**
     * Transfers coral from indexer to manipulator.
     * Used when coral is already indexed and ready to transfer.
     *
     * @param intake           The Intake subsystem
     * @param elevator         The Elevator subsystem
     * @param manipulator      The Manipulator subsystem
     * @param safeSubsystems   Safety-aware command utilities
     * @param pieceCombos      Command utilities for piece manipulation
     * @return A Command to transfer coral to the manipulator
     */
    public static Command transferCoral(Intake intake, Elevator elevator, Manipulator manipulator, SafeSubsystems safeSubsystems, PieceCombos pieceCombos) {
        return Commands.sequence(
            // Move elevator/manipulator to intake height
            safeSubsystems.safeMoveCommand(elevator.coralIntake(), manipulator.pivot.coralIntake(), ELEVATOR.CORAL.INTAKE_HEIGHT),
            // Run transfer and piece combo logic, only if coral is at indexer
            Commands.deadline(
                intake.transfer(),
                pieceCombos.intakeCoral()
            ).onlyIf(() -> intake.sensors.getCoralLocation() == CoralLocation.INDEXER)
        );
    }
}
