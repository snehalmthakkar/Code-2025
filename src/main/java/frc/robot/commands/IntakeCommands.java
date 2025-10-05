package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants.ELEVATOR;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeSensors.CoralLocation;
import frc.robot.subsystems.manipulator.Manipulator;

public final class IntakeCommands {
    private IntakeCommands() {
    }

    public static Command intakeTransfer(Intake intake, Elevator elevator, Manipulator manipulator, SafeSubsystems safeSubsystems, PieceCombos pieceCombos) {
        return Commands.sequence(
            Commands.deadline(
                Commands.parallel(
                    intake.rollers.intake(),
                    intake.pivot.deploy().until(() -> intake.sensors.getCoralLocation() == CoralLocation.INTAKE || intake.sensors.getCoralLocation() == CoralLocation.INDEXER)
                        .andThen(
                            Commands.waitUntil(() -> intake.sensors.getCoralLocation() == CoralLocation.INDEXER),
                            intake.pivot.stow()
                        ),
                    intake.indexer.intake()
                ).until(() -> intake.sensors.getCoralLocation() == CoralLocation.INDEXER),
                safeSubsystems.safeMoveCommand(elevator.coralIntake(), manipulator.pivot.coralIntake(), ELEVATOR.CORAL.INTAKE_HEIGHT)
            ).onlyIf(() -> !manipulator.grabber.hasCoral() && (intake.sensors.getCoralLocation() == CoralLocation.OUTSIDE || intake.sensors.getCoralLocation() == CoralLocation.INTAKE || intake.sensors.getCoralLocation() == CoralLocation.TRANSFER_TO_INDEXER)),
            Commands.deadline(
                safeSubsystems.safeMoveCommand(elevator.coralIntake(), manipulator.pivot.coralIntake(), ELEVATOR.CORAL.INTAKE_HEIGHT),
                intake.pivot.stow()
            ),
            Commands.deadline(
                intake.transfer(),
                pieceCombos.intakeCoral(),
                intake.pivot.stow()
            ).onlyIf(() -> intake.sensors.getCoralLocation() == CoralLocation.INDEXER || intake.sensors.getCoralLocation() == CoralLocation.TRANSFER_TO_MANIPULATOR || !manipulator.grabber.isCoralClear())
        );
    }

    public static Command intakeTransferAuto(Intake intake, Elevator elevator, Manipulator manipulator, SafeSubsystems safeSubsystems, PieceCombos pieceCombos) {
        if (RobotBase.isSimulation()) {
            return Commands.either(
                Commands.waitSeconds(2).andThen(() -> intake.sensors.simIntakeCoral()),
                Commands.waitUntil(() -> false),
                () -> Math.random() < 0.75
            ).onlyIf(() -> intake.sensors.getCoralLocation() == CoralLocation.OUTSIDE)
                .andThen(Commands.waitSeconds(0.5));
        }

        return Commands.sequence(
            Commands.deadline(
                Commands.parallel(
                    intake.rollers.intake(),
                    intake.pivot.deploy().until(() -> intake.sensors.getCoralLocation() == CoralLocation.INTAKE || intake.sensors.getCoralLocation() == CoralLocation.INDEXER),
                    intake.indexer.intake()
                ).until(() -> intake.sensors.getCoralLocation() == CoralLocation.INDEXER),
                safeSubsystems.safeMoveCommand(elevator.coralIntake(), manipulator.pivot.coralIntake(), ELEVATOR.CORAL.INTAKE_HEIGHT)
            ).onlyIf(() -> !manipulator.grabber.hasCoral() && (intake.sensors.getCoralLocation() == CoralLocation.OUTSIDE || intake.sensors.getCoralLocation() == CoralLocation.INTAKE || intake.sensors.getCoralLocation() == CoralLocation.TRANSFER_TO_INDEXER)),
            safeSubsystems.safeMoveCommand(elevator.coralIntake(), manipulator.pivot.coralIntake(), ELEVATOR.CORAL.INTAKE_HEIGHT),
            Commands.deadline(
                intake.transfer(),
                pieceCombos.intakeCoral()
            ).onlyIf(() -> intake.sensors.getCoralLocation() == CoralLocation.INDEXER || intake.sensors.getCoralLocation() == CoralLocation.TRANSFER_TO_MANIPULATOR || !manipulator.grabber.isCoralClear())
        );
    }

    public static Command intakeTransferVertical(Intake intake, Elevator elevator, Manipulator manipulator, SafeSubsystems safeSubsystems, PieceCombos pieceCombos) {
        return Commands.sequence(
            Commands.deadline(
                Commands.sequence(
                    Commands.parallel(
                        intake.rollers.intake(),
                        intake.pivot.deployVertical()
                    ).until(() -> intake.sensors.getCoralLocation() == CoralLocation.INTAKE),
                    intake.pivot.deploy(),
                    Commands.parallel(
                        intake.rollers.intake(),
                        intake.indexer.intake()
                    ).until(() -> intake.sensors.getCoralLocation() == CoralLocation.INDEXER)
                ),
                safeSubsystems.safeMoveCommand(elevator.coralIntake(), manipulator.pivot.coralIntake(), ELEVATOR.CORAL.INTAKE_HEIGHT)
            ).onlyIf(() -> !manipulator.grabber.hasCoral() && (intake.sensors.getCoralLocation() == CoralLocation.OUTSIDE || intake.sensors.getCoralLocation() == CoralLocation.TRANSFER_TO_INDEXER)),
            Commands.deadline(
                safeSubsystems.safeMoveCommand(elevator.coralIntake(), manipulator.pivot.coralIntake(), ELEVATOR.CORAL.INTAKE_HEIGHT),
                intake.pivot.stow()
            ),
            Commands.deadline(
                intake.transfer(),
                pieceCombos.intakeCoral(),
                intake.pivot.stow()
            ).onlyIf(() -> intake.sensors.getCoralLocation() == CoralLocation.INDEXER || intake.sensors.getCoralLocation() == CoralLocation.TRANSFER_TO_MANIPULATOR)
        );
    }

    public static Command intakeCoral(Intake intake, Elevator elevator, Manipulator manipulator, SafeSubsystems safeSubsystems, PieceCombos pieceCombos) {
        return Commands.deadline(
            intake.intake(),
            safeSubsystems.safeMoveCommand(elevator.coralIntake(), manipulator.pivot.coralIntake(), ELEVATOR.CORAL.INTAKE_HEIGHT)
        ).onlyIf(() -> RobotBase.isSimulation() || (!manipulator.grabber.hasCoral() && intake.sensors.getCoralLocation() == CoralLocation.OUTSIDE));
    }

    public static Command transferCoral(Intake intake, Elevator elevator, Manipulator manipulator, SafeSubsystems safeSubsystems, PieceCombos pieceCombos) {
        return Commands.sequence(
            safeSubsystems.safeMoveCommand(elevator.coralIntake(), manipulator.pivot.coralIntake(), ELEVATOR.CORAL.INTAKE_HEIGHT),
            Commands.deadline(
                intake.transfer(),
                pieceCombos.intakeCoral()
            ).onlyIf(() -> intake.sensors.getCoralLocation() == CoralLocation.INDEXER)
        );
    }
}
