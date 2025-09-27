package frc.robot.auto;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import com.team6962.lib.utils.CommandUtils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.IntakeCommands;
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

    public Command scorePreloadCoral(CoralPosition position) {
        return CommandUtils.annotate("Score coral on " + position, Commands.parallel(
            CommandUtils.annotate("Align to " + position, robot.autoAlign.alignPole(position.pole, true)),
            Commands.sequence(
                CommandUtils.annotate("Move to L" + position.level, CommandUtils.selectByMode(
                    robot.pieceCombos.coral(position.level),
                    Commands.waitSeconds(0.5)
                )),
                CommandUtils.annotate("Wait until aligned", Commands.waitUntil(
                    () -> robot.swerveDrive.isWithinToleranceOf(ReefPositioning.getCoralPlacePose(position.pole),
                    Inches.of(1),
                    Degrees.of(3)
                ))),
                CommandUtils.annotate("Drops coral", CommandUtils.selectByMode(
                    robot.manipulator.grabber.dropCoral(),
                    Commands.waitSeconds(0.25)
                ))
            )
        ).deadlineFor(robot.intake.pivot.deploy()));
    }

    public Command scoreCoral(CoralPosition position) {
        return CommandUtils.annotate("Score coral on " + position, Commands.parallel(
            CommandUtils.annotate("Align to " + position, robot.autoAlign.alignPole(position.pole, true)),
            Commands.sequence(
                CommandUtils.annotate("Intake transfer", IntakeCommands.intakeTransfer(robot.intake, robot.elevator, robot.manipulator, robot.safeties, robot.pieceCombos)),
                CommandUtils.annotate("Move to L" + position.level, CommandUtils.selectByMode(
                    robot.pieceCombos.coral(position.level),
                    Commands.waitSeconds(0.5)
                )),
                CommandUtils.annotate("Wait until aligned", Commands.waitUntil(
                    () -> robot.swerveDrive.isWithinToleranceOf(ReefPositioning.getCoralPlacePose(position.pole),
                    Inches.of(1),
                    Degrees.of(3)
                ))),
                CommandUtils.annotate("Drops coral", CommandUtils.selectByMode(
                    robot.manipulator.grabber.dropCoral(),
                    Commands.waitSeconds(0.25)
                ))
            )
        ));
    }

    public Command intakeCoral(CoralStation coralStation) {
        return CommandUtils.annotate("Intake coral from " + coralStation + " coral station", Commands.deadline(
            CommandUtils.annotate("Intake coral", IntakeCommands.intakeCoral(robot.intake, robot.elevator, robot.manipulator, robot.safeties, robot.pieceCombos)
                .withDeadline(CommandUtils.selectByMode(
                    Commands.waitUntil(() -> robot.intake.sensors.getCoralLocation() == CoralLocation.INTAKE),
                    Commands.waitUntil(() -> robot.swerveDrive.isWithinToleranceOf(StationPositioning.getGroundIntakePose(coralStation), Inches.of(6), Degrees.of(15)))
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
}
