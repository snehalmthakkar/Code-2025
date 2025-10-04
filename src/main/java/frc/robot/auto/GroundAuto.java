package frc.robot.auto;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Set;
import java.util.function.BooleanSupplier;

import com.team6962.lib.utils.CommandUtils;
import com.team6962.lib.utils.KinematicsUtils;

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

        if (RobotBase.isSimulation()) {
            return Commands.waitSeconds(0.5);
        }

        Command elevatorCommand = fast && level >= 3 ? robot.elevator.ready() : robot.elevator.coral(level);
        Command pivotCommand = level == 4 ? robot.manipulator.pivot.safe() : robot.manipulator.placeCoralL23();

        return CommandUtils.annotate("Ready coral for L" + level, Commands.parallel(elevatorCommand, pivotCommand));
    }

    public Command scoreCoralWithMechanisms(int level) {
        if (level <= 1 || level > 4) {
            throw new IllegalArgumentException("Level must be between 2 and 4");
        }

        if (RobotBase.isSimulation()) {
            return Commands.waitSeconds(0.5);
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

    public Command intakeCoralFromStation(CoralStation coralStation, boolean approachSlow) {
        return CommandUtils.annotate("Intake coral from " + coralStation + " coral station", Commands.deadline(
            CommandUtils.selectByMode(
                Commands.waitUntil(() -> robot.intake.sensors.getCoralLocation() == CoralLocation.INTAKE),
                Commands.waitUntil(() -> robot.swerveDrive.isWithinToleranceOf(StationPositioning.getGroundStartIntakePose(coralStation), Inches.of(12), Degrees.of(30)))
                    .andThen(Commands.defer(() -> Commands.waitSeconds(Math.pow(Math.random(), 1.5) * Math.signum(Math.random() - 0.5) + 1.25), Set.of()))
            ),
            CommandUtils.annotate("Intake coral", IntakeCommands.intakeCoral(robot.intake, robot.elevator, robot.manipulator, robot.safeties, robot.pieceCombos)),
            CommandUtils.annotate(
                "Drive over to intake from " + coralStation + " coral station",
                approachSlow ? robot.swerveDrive.driveQuicklyTo(StationPositioning.getGroundStartIntakePose(coralStation), MetersPerSecond.of(2)) :
                robot.swerveDrive.driveQuicklyTo(StationPositioning.getGroundStartIntakePose(coralStation))
            ).andThen(Commands.repeatingSequence(
                CommandUtils.annotate(
                    "Drive beside coral station",
                    robot.swerveDrive.driveQuicklyTo(StationPositioning.getGroundEndIntakePose(coralStation), MetersPerSecond.of(2))
                ),
                CommandUtils.annotate(
                    "Drive over to intake from " + coralStation + " coral station",
                    robot.swerveDrive.driveQuicklyTo(StationPositioning.getGroundStartIntakePose(coralStation))
                )
            ))
        ));
    }

    public Command sideAutonomous(CoralStation coralStation) {
        boolean reflect = coralStation == CoralStation.LEFT;

        return CommandUtils.annotate(coralStation + " Side", Commands.sequence(
            scorePreloadCoral(new CoralPosition(10, 4).reflectedIf(reflect), false),
            robot.swerveDrive.driveQuicklyTo(
                StationPositioning.reflectPose(new Pose2d(4.3, 2.42, Rotation2d.fromDegrees(120)), reflect),
                KinematicsUtils.createChassisSpeeds(new Translation2d(2, Rotation2d.fromDegrees(210).times(reflect ? -1 : 1)), new Rotation2d())
            ).deadlineFor(robot.intake.pivot.deploy()),
            intakeCoralFromStation(coralStation, true),
            scoreCoral(new CoralPosition(7, 4).reflectedIf(reflect)),
            intakeCoralFromStation(coralStation, false),
            scoreCoral(new CoralPosition(8, 4).reflectedIf(reflect)),
            intakeCoralFromStation(coralStation, false),
            scoreCoral(new CoralPosition(9, 4).reflectedIf(reflect)),
            intakeCoralFromStation(coralStation, false),
            scoreCoral(new CoralPosition(7, 3).reflectedIf(reflect))
        ));
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

    public static Translation2d LOLLIPOP_1 = new Translation2d(1.27, 2.2);
    public static Translation2d LOLLIPOP_2 = new Translation2d(1.27, 4.025);
    public static Translation2d LOLLIPOP_3 = new Translation2d(1.27, 5.85);

    public Command intakeLollipop(int id, boolean finalLollipop) {
        return Commands.defer(() -> {
            Translation2d currentPosition = robot.swerveDrive.getEstimatedPose().getTranslation();
            
            Translation2d lollipopPosition = id == 0 ? LOLLIPOP_1 : id == 1 ?
            LOLLIPOP_2 : LOLLIPOP_3;

            Rotation2d angleToLollipop = currentPosition.minus(lollipopPosition).getAngle();

            Pose2d approachPose = new Pose2d(lollipopPosition.plus(new Translation2d(Units.inchesToMeters(50), angleToLollipop)), angleToLollipop);
            Pose2d finalPose = new Pose2d(lollipopPosition.minus(new Translation2d(0.3, angleToLollipop)), angleToLollipop);

            ChassisSpeeds approachSpeeds = new ChassisSpeeds(-0.5 * angleToLollipop.getCos(), -0.5 * angleToLollipop.getSin(), 0);
            ChassisSpeeds finalSpeeds = new ChassisSpeeds(0 * angleToLollipop.getCos(), 0 * angleToLollipop.getSin(), 0);

            return Commands.deadline(
                robot.swerveDrive.driveQuicklyTo(approachPose, approachSpeeds)
                        .andThen(robot.swerveDrive.driveQuicklyTo(finalPose, finalSpeeds, MetersPerSecond.of(1.25)))
                        .andThen(finalLollipop ? Commands.waitUntil(() -> false) : Commands.waitSeconds(1)),
                IntakeCommands.intakeTransferAuto(robot.intake, robot.elevator, robot.manipulator, robot.safeties, robot.pieceCombos)
            ).until(() -> robot.intake.sensors.getCoralLocation() == CoralLocation.INTAKE);
        }, Set.of(robot.intake.indexer, robot.intake.pivot, robot.intake.rollers, robot.elevator, robot.manipulator.grabber, robot.manipulator.pivot, robot.swerveDrive));
    }

    public Command recoverFromLollipopFailure(int failedId, int nextId) {
        return Commands.defer(() -> {
            // Translation2d failedLollipop = failedId == 0 ? LOLLIPOP_1 : failedId == 1 ?
            // LOLLIPOP_2 : LOLLIPOP_3;

            Translation2d nextLollipop = nextId == 0 ? LOLLIPOP_1 : nextId == 1 ?
            LOLLIPOP_2 : LOLLIPOP_3;

            Rotation2d angleToNextLollipop = Rotation2d.fromDegrees(-30).times(Math.signum(nextId - failedId));

            Translation2d setupTranslation = new Translation2d(nextLollipop.getX(), nextLollipop.getY()).plus(new Translation2d(Units.inchesToMeters(50), angleToNextLollipop));

            Pose2d setupPose = new Pose2d(setupTranslation, angleToNextLollipop);

            ChassisSpeeds setupSpeeds = new ChassisSpeeds(0.5 * angleToNextLollipop.getCos(), 0.5 * angleToNextLollipop.getSin(), 0);
            // ChassisSpeeds recoverSpeeds = new ChassisSpeeds(1.5, 2 * Math.signum(nextId - failedId), 0);

            return Commands.parallel(
                robot.swerveDrive.driveQuicklyTo(setupPose, setupSpeeds),
                CommandUtils.selectByMode(
                    robot.intake.pivot.stow().andThen(robot.intake.pivot.deploy()),
                    Commands.waitSeconds(0.5)
                )
            );
        }, Set.of(robot.intake.indexer, robot.intake.pivot, robot.intake.rollers, robot.elevator, robot.manipulator.grabber, robot.manipulator.pivot, robot.swerveDrive));
    }

    public BooleanSupplier successfullyIntaked() {
        return () -> robot.intake.sensors.getCoralLocation() != CoralLocation.OUTSIDE;
    }

    public BooleanSupplier failedToIntake() {
        return () -> robot.intake.sensors.getCoralLocation() == CoralLocation.OUTSIDE;
    }

    public Command lollipopAuto(CoralStation coralStation, boolean scoreL4) {
        boolean reflect = coralStation == CoralStation.LEFT;

        int lollipop1 = coralStation == CoralStation.LEFT ? 2 : 0;
        int lollipop2 = 1;
        int lollipop3 = coralStation == CoralStation.LEFT ? 0 : 2;

        return CommandUtils.annotate(coralStation + " " + (scoreL4 ? "Side & " : "") + "Lollipop", Commands.sequence(
            prepareLollipops(coralStation, scoreL4),
            scorePreloadCoral((scoreL4 ? new CoralPosition(7, 4) : new CoralPosition(6, 4)).reflectedIf(reflect), scoreL4),
            intakeLollipop(lollipop1, false),
            Commands.either(
                scoreL4 ? scoreCoral(new CoralPosition(6, 4).reflectedIf(reflect)) : scoreCoral(new CoralPosition(5, 4).reflectedIf(reflect)),
                recoverFromLollipopFailure(lollipop1, lollipop2),
                successfullyIntaked()
            ),
            intakeLollipop(lollipop2, false),
            Commands.either(
                scoreCoral(new CoralPosition(6, 2).reflectedIf(reflect)),
                recoverFromLollipopFailure(lollipop2, lollipop3),
                successfullyIntaked()
            ),
            intakeLollipop(lollipop3, true),
            scoreCoral(new CoralPosition(5, 2).reflectedIf(reflect)).onlyIf(successfullyIntaked())
        ));
    }

    private Command pickupAlgae(int face) {
        int level = ReefPositioning.getAlgaeHeight(face);

        return Commands.sequence(
            Commands.deadline(
                CommandUtils.selectByMode(
                    robot.manipulator.pivot.stow().until(() -> robot.manipulator.pivot.getPosition().gt(Degrees.of(-10))),
                    Commands.waitSeconds(0.5)
                ),
                robot.swerveDrive
                    .driveQuicklyTo(ReefPositioning.getAlgaeAlignPose(face)),
                robot.elevator.algae(level)
            ),
            Commands.parallel(
                robot.swerveDrive.driveTo(ReefPositioning.getAlgaePickupPose(face)),
                CommandUtils.selectByMode(
                    Commands.sequence(
                        robot.pieceCombos.algae(level),
                        Commands.parallel(
                            robot.manipulator.grabber.intakeAlgae(),
                            robot.manipulator.pivot.algaeReef()
                        )
                    ),
                    Commands.waitSeconds(0.5)
                )
            ).withDeadline(CommandUtils.selectByMode(
                Commands.waitUntil(robot.manipulator.grabber::hasAlgae),
                Commands.waitSeconds(0.5)
            )),
            robot.swerveDrive.driveQuicklyTo(ReefPositioning.getAlgaeAlignPose(face)).deadlineFor(robot.manipulator.pivot.stow())
        );
    }

    private Command scoreAlgae() {
        return Commands.sequence(
            robot.autoAlign.autoAlignSetupBarge().deadlineFor(robot.elevator.stow(), robot.manipulator.pivot.stow()),
            CommandUtils.selectByMode(
                Commands.parallel(
                    robot.elevator.stow(),
                    robot.manipulator.pivot.safe()
                ),
                Commands.waitSeconds(0.1)
            ),
            CommandUtils.selectByMode(
                robot.pieceCombos.stow()
                    .andThen(robot.elevator.launchBarge())
                    .withDeadline(Commands.sequence(
                        Commands.waitUntil(() -> robot.elevator.getPosition().gt(Inches.of(52.5))),
                        robot.manipulator.grabber.dropAlgae().withTimeout(0.5)
                    )),
                Commands.waitSeconds(1)
            )
        );
    }

    public Command middleAuto(int algaeCount) {
        return CommandUtils.annotate(algaeCount > 0 ? "Middle Coral & " + algaeCount + " Algae" : "Middle Coral", Commands.sequence(
            Commands.either(
                scorePreloadCoral(new CoralPosition(0, 4), false),
                scorePreloadCoral(new CoralPosition(11, 4), false),
                () -> robot.swerveDrive.getEstimatedPose().getY() > ReefPositioning.REEF_CENTER.getY()
            ),
            Commands.sequence(
                pickupAlgae(0),
                scoreAlgae()
            ).onlyIf(() -> algaeCount >= 1),
            Commands.sequence(
                robot.swerveDrive.driveQuicklyTo(new Pose2d(6.3, 5.25, Rotation2d.fromDegrees(-40))).deadlineFor(robot.pieceCombos.algaeL3()),
                robot.swerveDrive.driveQuicklyTo(new Pose2d(5.6, 5.5, Rotation2d.fromDegrees(-120))).deadlineFor(robot.pieceCombos.algaeL3()),
                pickupAlgae(1),
                scoreAlgae()
            ).onlyIf(() -> algaeCount >= 2)
        ));
    }
}
