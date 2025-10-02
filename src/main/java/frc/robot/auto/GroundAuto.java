package frc.robot.auto;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Set;

import com.team6962.lib.utils.CommandUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.IntakeCommands;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.ELEVATOR;
import frc.robot.field.Field;
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

    public Command readyMechanismsForScoreCoral(int level, boolean fast) {
        if (level <= 1 || level > 4) {
            throw new IllegalArgumentException("Level must be between 2 and 4");
        }

        Command elevatorCommand = fast && level >= 3 ? robot.elevator.ready() : robot.elevator.coral(level);
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

    public Command scorePreloadCoral(CoralPosition position, boolean scoreL4) {
        return CommandUtils.annotate("Score coral on " + position, Commands.deadline(
            Commands.sequence(
                CommandUtils.annotate("Ready for score on L" + position.level, readyMechanismsForScoreCoral(position.level, !scoreL4)),
                CommandUtils.annotate("Wait until aligned", Commands.waitUntil(
                    () -> robot.swerveDrive.isWithinToleranceOf(ReefPositioning.getCoralPlacePose(position.pole),
                    Inches.of(1),
                    Degrees.of(3)
                ))),
                CommandUtils.annotate("Score coral on L" + position.level, scoreCoralWithMechanisms(position.level))
            ),
            CommandUtils.annotate("Align to " + position, robot.autoAlign.alignPole(position.pole, false))
        ).deadlineFor(robot.intake.pivot.deploy()));
    }

    public Command scoreCoral(CoralPosition position) {
        return CommandUtils.annotate("Score coral on " + position, Commands.deadline(
            Commands.sequence(
                CommandUtils.annotate("Intake transfer", IntakeCommands.intakeTransferAuto(robot.intake, robot.elevator, robot.manipulator, robot.safeties, robot.pieceCombos)),
                CommandUtils.annotate("Ready for score on L" + position.level, readyMechanismsForScoreCoral(position.level, false).alongWith(
                    CommandUtils.annotate("Reposition coral", robot.manipulator.grabber.repositionCoral())
                )),
                CommandUtils.annotate("Wait until aligned", Commands.waitUntil(
                    () -> robot.swerveDrive.isWithinToleranceOf(ReefPositioning.getCoralPlacePose(position.pole),
                    Inches.of(1.5),
                    Degrees.of(3)
                ))),
                CommandUtils.annotate("Score coral on L" + position.level, scoreCoralWithMechanisms(position.level))
            ),
            CommandUtils.annotate("Align to " + position, robot.autoAlign.alignPole(position.pole, false))
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
                CommandUtils.annotate("Drive to intake from " + coralStation + " coral station", autoPickupCoral(StationPositioning.getGroundIntakePose(coralStation)))
        ));
    }

    public Command sideAutonomous(CoralStation coralStation) {
        boolean reflect = coralStation == CoralStation.LEFT;

        return CommandUtils.annotate(coralStation + " Side Autonomous", Commands.sequence(
            scorePreloadCoral(new CoralPosition(10, 4).reflectedIf(reflect), false),
            intakeCoral(coralStation),
            scoreCoral(new CoralPosition(7, 4).reflectedIf(reflect)),
            intakeCoral(coralStation),
            scoreCoral(new CoralPosition(8, 4).reflectedIf(reflect)),
            intakeCoral(coralStation),
            scoreCoral(new CoralPosition(9, 4).reflectedIf(reflect)),
            intakeCoral(coralStation)
        ));
    }

    public Command autoPickupCoral(Pose2d defaultPose) {
        return robot.swerveDrive.driveQuicklyTo(() ->
            robot.coralDetection.getCoralLocation() == null ? defaultPose :
            new Pose2d(
                robot.coralDetection.getCoralLocation(),
                robot.coralDetection.getCoralLocation().minus(robot.swerveDrive.getEstimatedPose().getTranslation()).getAngle()
            )
        );
    }

    public Command prepareLollipops(CoralStation coralStation, boolean scoreL4) {
        Pose2d pose = scoreL4 ? new Pose2d(4.36, 2.02, Rotation2d.fromDegrees(60)) : new Pose2d(2.96, 1.36, Rotation2d.fromDegrees(0));
        ChassisSpeeds speeds = scoreL4 ? new ChassisSpeeds(-1, 0.5, 0) : new ChassisSpeeds(-1.5, 0.75, 0);
        
        if (coralStation == CoralStation.LEFT) {
            pose = new Pose2d(pose.getX(), Field.WIDTH - pose.getY(), pose.getRotation().unaryMinus());
            speeds = new ChassisSpeeds(speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, -speeds.omegaRadiansPerSecond);
          }

        return robot.swerveDrive.driveQuicklyTo(pose, speeds);
    }

    public Command intakeLollipop(int id) {
        return Commands.defer(() -> {
            Translation2d currentPosition = robot.swerveDrive.getEstimatedPose().getTranslation();
            
            Translation2d lollipopPosition = id == 0 ? new Translation2d(1.27, 2.2)
            : id == 1 ? new Translation2d(1.27, 4.025) :
            new Translation2d(1.27, 5.85);

            Rotation2d angleToLollipop = currentPosition.minus(lollipopPosition).getAngle();

            Pose2d finalPose = new Pose2d(lollipopPosition, angleToLollipop);
            Pose2d intermediatePose = new Pose2d(lollipopPosition.plus(new Translation2d(Units.inchesToMeters(50), angleToLollipop)), angleToLollipop);
            Pose2d exitPose = new Pose2d(lollipopPosition.minus(new Translation2d(0.3, angleToLollipop)), angleToLollipop);

            // ChassisSpeeds finalSpeeds = new ChassisSpeeds(-0.1 * angleToLollipop.getCos(), -0.1 * angleToLollipop.getSin(), 0);
            ChassisSpeeds intermediateSpeeds = new ChassisSpeeds(-0.25 * angleToLollipop.getCos(), -0.5 * angleToLollipop.getSin(), 0);
            ChassisSpeeds exitSpeeds = new ChassisSpeeds(0 * angleToLollipop.getCos(), 0 * angleToLollipop.getSin(), 0);
        
            return Commands.parallel(
                IntakeCommands.intakeTransferAuto(robot.intake, robot.elevator, robot.manipulator, robot.safeties, robot.pieceCombos),
                robot.swerveDrive.driveQuicklyTo(intermediatePose, intermediateSpeeds)
                    // .andThen(robot.swerveDrive.driveQuicklyTo(finalPose, finalSpeeds))
                    .andThen(robot.swerveDrive.driveQuicklyTo(exitPose, exitSpeeds, MetersPerSecond.of(1.25)))
            ).until(() -> robot.intake.sensors.getCoralLocation() == CoralLocation.INTAKE);
        }, Set.of(robot.intake.indexer, robot.intake.pivot, robot.intake.rollers, robot.elevator, robot.manipulator.grabber, robot.manipulator.pivot, robot.swerveDrive));
    }

    public Command lollipopAuto(CoralStation coralStation, boolean scoreL4) {
        boolean reflect = coralStation == CoralStation.LEFT;

        return CommandUtils.annotate(coralStation + " Side Lollipop", Commands.sequence(
            prepareLollipops(coralStation, scoreL4),
            scorePreloadCoral(new CoralPosition(7, 4).reflectedIf(reflect), scoreL4),
            intakeLollipop(coralStation == CoralStation.LEFT ? 2 : 0),
            scoreL4 ? scoreCoral(new CoralPosition(6, 4).reflectedIf(reflect)) : scoreCoral(new CoralPosition(6, 2).reflectedIf(reflect)),
            intakeLollipop(1),
            scoreCoral(new CoralPosition(5, 4).reflectedIf(reflect)),
            intakeLollipop(coralStation == CoralStation.LEFT ? 0 : 2),
            scoreCoral(new CoralPosition(5, 2).reflectedIf(reflect))
        ));
    }
}
