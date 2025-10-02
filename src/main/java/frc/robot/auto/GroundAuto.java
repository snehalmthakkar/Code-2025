package frc.robot.auto;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import com.team6962.lib.utils.CommandUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.IntakeCommands;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.ELEVATOR;
import frc.robot.field.ReefPositioning;
import frc.robot.field.ReefPositioning.CoralPosition;
import frc.robot.field.StationPositioning;
import frc.robot.field.StationPositioning.CoralStation;
import frc.robot.subsystems.intake.IntakeSensors.CoralLocation;

public class GroundAuto {
    private RobotContainer robot;

    public GroundAuto(RobotContainer robot) {
        this.robot = robot;
    }

    public Command readyMechanismsForScoreCoral(int level) {
        if (level <= 1 || level > 4) {
            throw new IllegalArgumentException("Level must be between 2 and 4");
        }

        Command elevatorCommand = level >= 3 ? robot.elevator.ready().until(() -> robot.elevator.getPosition().gt(ELEVATOR.AUTO.READY_HEIGHT)) : robot.elevator.coralL2();
        Command pivotCommand = level == 4 ? robot.manipulator.pivot.safe() : robot.manipulator.placeCoralL23();

        return CommandUtils.annotate("Ready coral for L" + level, RobotBase.isReal() ? Commands.parallel(elevatorCommand, pivotCommand) : elevatorCommand);
    }

    public Command scoreCoralWithMechanisms(int level) {
        if (level <= 1 || level > 4) {
            throw new IllegalArgumentException("Level must be between 2 and 4");
        }
        
        if (level == 4) {
            return Commands.sequence(
                robot.elevator.coralL4(),
                RobotBase.isReal() ? robot.manipulator.placeCoralL4() : Commands.none(),
                robot.manipulator.grabber.dropCoral()
            );
        } else {
            return Commands.sequence(
                Commands.parallel(
                    robot.elevator.coral(level),
                    RobotBase.isReal() ? robot.manipulator.placeCoralL23() : Commands.none()
                ),
                robot.manipulator.grabber.dropCoral()
            );
        }
    }

    public Command scorePreloadCoral(CoralPosition position) {
        return CommandUtils.annotate("Score coral on " + position, Commands.parallel(
            CommandUtils.annotate("Align to " + position, robot.autoAlign.alignPole(position.pole, true)),
            Commands.sequence(
                CommandUtils.annotate("Ready for score on L" + position.level, readyMechanismsForScoreCoral(position.level)),
                CommandUtils.annotate("Wait until aligned", Commands.waitUntil(
                    () -> robot.swerveDrive.isWithinToleranceOf(ReefPositioning.getCoralPlacePose(position.pole),
                    Inches.of(1),
                    Degrees.of(3)
                ))),
                CommandUtils.annotate("Score coral on L" + position.level, scoreCoralWithMechanisms(position.level))
            )
        ).deadlineFor(robot.intake.pivot.deploy()));
    }

    public Command scoreCoral(CoralPosition position) {
        return CommandUtils.annotate("Score coral on " + position, Commands.parallel(
            CommandUtils.annotate("Align to " + position, robot.autoAlign.alignPole(position.pole, true)),
            Commands.sequence(
                CommandUtils.annotate("Intake transfer", IntakeCommands.intakeTransfer(robot.intake, robot.elevator, robot.manipulator, robot.safeties, robot.pieceCombos)),
                CommandUtils.annotate("Ready for score on L" + position.level, readyMechanismsForScoreCoral(position.level)),
                CommandUtils.annotate("Wait until aligned", Commands.waitUntil(
                    () -> robot.swerveDrive.isWithinToleranceOf(ReefPositioning.getCoralPlacePose(position.pole),
                    Inches.of(1),
                    Degrees.of(3)
                ))),
                CommandUtils.annotate("Score coral on L" + position.level, scoreCoralWithMechanisms(position.level))
            )
        ));
    }

    public Command intakeCoral(CoralStation coralStation) {
        return CommandUtils.annotate("Intake coral from " + coralStation + " coral station", Commands.deadline(
            CommandUtils.annotate("Intake coral", IntakeCommands.intakeCoral(robot.intake, robot.elevator, robot.manipulator, robot.safeties, robot.pieceCombos)
                .withDeadline(CommandUtils.selectByMode(
                    Commands.waitUntil(() -> robot.intake.sensors.getCoralLocation() == CoralLocation.INTAKE),
                    Commands.waitUntil(() -> robot.swerveDrive.isWithinToleranceOf(StationPositioning.getGroundIntakePose(coralStation), Inches.of(6), Degrees.of(15)) &&
                        robot.elevator.isNear(Constants.ELEVATOR.CORAL.INTAKE_HEIGHT))
                ))),
                CommandUtils.annotate("Drive to intake from " + coralStation + " coral station", robot.swerveDrive.driveTo(StationPositioning.getGroundIntakePose(coralStation)))
        ));
    }

    public Command sideAutonomous(CoralStation coralStation) {
        boolean reflect = coralStation == CoralStation.LEFT;

        return CommandUtils.annotate(coralStation + " Side Autonomous", Commands.sequence(
            scorePreloadCoral(new CoralPosition(10, 4).reflectedIf(reflect)),
            intakeCoral(coralStation),
            scoreCoral(new CoralPosition(7, 4).reflectedIf(reflect)),
            intakeCoral(coralStation),
            scoreCoral(new CoralPosition(8, 4).reflectedIf(reflect)),
            intakeCoral(coralStation),
            scoreCoral(new CoralPosition(9, 4).reflectedIf(reflect)),
            intakeCoral(coralStation)
        ));
    }

    // public Command prepareLollipops(CoralStation coralStation) {
    //     Pose2d lollipopPose = new Pose2d(2.96, 1.36, Rotation2d.fromDegrees(0));
    // }
}
