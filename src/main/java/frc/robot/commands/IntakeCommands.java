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
        return Commands.deadline(
            intake.intake(),
            safeSubsystems.safeMoveCommand(elevator.coralIntake(), manipulator.pivot.coralIntake(), ELEVATOR.CORAL.INTAKE_HEIGHT)
        ).onlyIf(() -> !manipulator.grabber.hasCoral() && intake.sensors.getCoralLocation() != CoralLocation.OUTSIDE).andThen(Commands.deadline(
            intake.transfer(),
            manipulator.pivot.coralIntake()
        ).onlyIf(() -> intake.sensors.getCoralLocation() == CoralLocation.INDEXER));
    }

    public static Command intakeCoral(Intake intake, Elevator elevator, Manipulator manipulator, SafeSubsystems safeSubsystems, PieceCombos pieceCombos) {
        return Commands.deadline(
            intake.intake(),
            safeSubsystems.safeMoveCommand(elevator.coralIntake(), manipulator.pivot.coralIntake(), ELEVATOR.CORAL.INTAKE_HEIGHT)
        ).onlyIf(() -> RobotBase.isSimulation() || (!manipulator.grabber.hasCoral() && intake.sensors.getCoralLocation() != CoralLocation.OUTSIDE));
    }

    public static Command transferCoral(Intake intake, Elevator elevator, Manipulator manipulator, SafeSubsystems safeSubsystems, PieceCombos pieceCombos) {
        return Commands.deadline(
            intake.transfer(),
            manipulator.pivot.coralIntake()
        ).onlyIf(() -> intake.sensors.getCoralLocation() == CoralLocation.INDEXER);
    }
}
