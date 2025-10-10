/**
 * Autonomous.java
 *
 * Defines autonomous routines for the robot, including side and middle autos, using SwerveDrive,
 * manipulator, elevator, and piece combo subsystems. Provides logic for preparing start poses, scoring,
 * intaking, and coordinating complex sequences for the autonomous period.
 */

package frc.robot.auto;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Seconds;

import com.team6962.lib.swerve.SwerveDrive;
import com.team6962.lib.utils.CommandUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.PieceCombos;
import frc.robot.constants.Constants.ELEVATOR;
import frc.robot.field.Field;
import frc.robot.field.ReefPositioning;
import frc.robot.field.ReefPositioning.CoralPosition;
import frc.robot.field.StationPositioning;
import frc.robot.field.StationPositioning.CoralStation;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.manipulator.Manipulator;

/**
 * Provides autonomous routines and utilities for the robot, supporting side and middle autos,
 * intake and scoring sequences, and pose preparation.
 */
public class Autonomous {
  private SwerveDrive swerveDrive;
  private Manipulator manipulator;
  private Elevator elevator;
  private PieceCombos pieceCombos;

  /**
   * Constructs an Autonomous instance with subsystem references.
   *
   * @param swerveDrive  The swerve drive subsystem
   * @param manipulator  The manipulator subsystem
   * @param elevator     The elevator subsystem
   * @param pieceCombos  The piece combo command helper
   */
  public Autonomous(
      SwerveDrive swerveDrive,
      Manipulator manipulator,
      Elevator elevator,
      PieceCombos pieceCombos) {
    this.swerveDrive = swerveDrive;
    this.manipulator = manipulator;
    this.elevator = elevator;
    this.pieceCombos = pieceCombos;
  }

  /**
   * Enum for specifying side (LEFT or RIGHT) for autonomous routines.
   */
  public enum Side {
    LEFT("left", CoralStation.LEFT),
    RIGHT("right", CoralStation.RIGHT);

    private final String id;
    private final CoralStation station;

    Side(String id, CoralStation station) {
      this.id = id;
      this.station = station;
    }
  }

  /**
   * Returns the starting pose for side autonomous routines.
   *
   * @param side The autonomous side (LEFT or RIGHT)
   * @return The starting Pose2d
   */
  public Pose2d getSideAutonomousStartPose(Side side) {
    double leftSideY = 5.814218521118164;
    double leftSideRotationRads = -2.51279668554423;

    return new Pose2d(
        7.075444221496582,
        side == Side.LEFT ? leftSideY : (Field.WIDTH - leftSideY),
        Rotation2d.fromRadians(side == Side.LEFT ? leftSideRotationRads : -leftSideRotationRads));
  }

  /**
   * Returns the starting pose for middle autonomous.
   *
   * @return The starting Pose2d for middle auto
   */
  public Pose2d getMiddleAutonomousStartPose() {
    return new Pose2d(7.23, 4.19, Rotation2d.fromDegrees(180));
  }

  /**
   * Returns alternate starting pose for middle/side autonomous.
   *
   * @return The alternate Pose2d for middle/side auto
   */
  public Pose2d getMiddleSideAutonomousStartPose() {
    return new Pose2d(7.23, 4.026, Rotation2d.fromDegrees(180));
  }

  /**
   * Prepares the robot for autonomous by pathfinding and driving to the start pose.
   *
   * @param startPose The pose to move to before starting auto
   * @return Command to move the robot to the start pose
   */
  public Command prepareAutonomous(Pose2d startPose) {
    return Commands.sequence(
        swerveDrive.pathfindTo(startPose),
        swerveDrive
            .drivePreciselyTo(startPose)
            .withDeadline(
                Commands.sequence(
                    Commands.waitUntil(
                        () ->
                            swerveDrive.isWithinToleranceOf(
                                startPose, Inches.of(0.5), Degrees.of(3))))));
  }

  /**
   * Creates a side autonomous routine with a complex sequence of intake and place actions.
   *
   * @param side           The side (LEFT/RIGHT)
   * @param startInMiddle  Whether to use the alternate start position
   * @return Command for the full side autonomous routine
   */
  public Command createSideAutonomous(Side side, boolean startInMiddle) {
    return Commands.sequence(
        swerveDrive
            .followChoreoPath("side-place-0-" + side.id + (startInMiddle ? "-middle" : ""))
            .deadlineFor(elevator.ready().repeatedly()),
        placeCoral(new CoralPosition(side == Side.LEFT ? 1 : 10, 4)),
        swerveDrive
            .followChoreoPath("side-intake-1-" + side.id)
            .deadlineFor(pieceCombos.intakeCoral()),
        intakeCoral(side.station, 1, Seconds.of(0.75)),
        swerveDrive
            .followChoreoPath("side-place-1-" + side.id)
            .deadlineFor(intakeThenRaiseElevator()),
        placeCoral(new CoralPosition(side == Side.LEFT ? 4 : 7, 4)),
        swerveDrive
            .followChoreoPath("side-intake-2-" + side.id)
            .deadlineFor(pieceCombos.intakeCoral()),
        intakeCoral(side.station, 1, Seconds.of(0.75)),
        swerveDrive
            .followChoreoPath("side-place-2-" + side.id)
            .deadlineFor(intakeThenRaiseElevator()),
        placeCoral(new CoralPosition(side == Side.LEFT ? 3 : 8, 4)));
  }

  /** The pose on the field for throwing algae onto the barge. */
  private static Pose2d BARGE_THROW_POSE =
      new Pose2d(7.408452033996582, 4.844626426696777, Rotation2d.fromDegrees(0));

  /**
   * Creates the middle autonomous routine, including coral and algae pickup/placement.
   *
   * @return Command for the full middle autonomous routine
   */
  public Command createMiddleAutonomous() {
    return Commands.sequence(
        Commands.waitSeconds(1),
        CommandUtils.annotate("middle-coral-place", swerveDrive.followChoreoPath("middle-coral-place"))
            .deadlineFor(elevator.ready().repeatedly()),
        CommandUtils.annotate("place coral", placeCoral(new CoralPosition(0, 4))),
        CommandUtils.annotate(
            "safe manipulator pivot",
            manipulator.pivot.safe()),
        CommandUtils.annotate("pickup algae", pickupAlgae(0)),
        CommandUtils.annotate("middle-algae-place", swerveDrive.followChoreoPath("middle-algae-place")),
        CommandUtils.annotate(
            "align to barge",
            swerveDrive
                .drivePreciselyTo(BARGE_THROW_POSE) // Barge throw pose
                .until(
                    () ->
                        swerveDrive.isWithinToleranceOf(
                            BARGE_THROW_POSE, Inches.of(4), Degrees.of(3)))),
        CommandUtils.annotate(
            "algae barge setup",
            CommandUtils.selectByMode(pieceCombos.algaeBargeSetup(), Commands.waitSeconds(0.5))),
        CommandUtils.annotate(
            "algae barge shoot",
            CommandUtils.selectByMode(pieceCombos.algaeBargeShoot(), Commands.waitSeconds(0.5))),
        Commands.sequence(
                CommandUtils.annotate(
                    "lower elevator",
                    CommandUtils.selectByMode(
                        Commands.waitUntil(
                            () -> elevator.getPosition().lt(ELEVATOR.CORAL.L4_HEIGHT)),
                        Commands.waitSeconds(0.25))))
            .alongWith(pieceCombos.stow()));
  }

  /**
   * Handles picking up an algae game piece from a specific field face.
   *
   * @param face The field face index for algae pickup
   * @return Command for picking up algae
   */
  private Command pickupAlgae(int face) {
    Pose2d alignPose = ReefPositioning.getAlgaeAlignPose(face);
    Pose2d pickupPose = ReefPositioning.getAlgaePickupPose(face);

    return Commands.sequence(
        CommandUtils.annotate("manipulator pivot safe", manipulator.pivot.safe()),
        Commands.parallel(
            CommandUtils.annotate(
                "twist to align",
                swerveDrive
                    .drivePreciselyTo(alignPose)
                    .until(
                        () ->
                            swerveDrive.isWithinToleranceOf(
                                alignPose, Inches.of(2), Degrees.of(6)))),
            CommandUtils.annotate(
                "elevator to algae l2",
                CommandUtils.selectByMode(
                    elevator.algaeL2(),
                    Commands.waitSeconds(0.5)))),
        CommandUtils.annotate(
            "manipulator to algae l2",
            Commands.deadline(
                CommandUtils.selectByMode(manipulator.pivot.algaeReef(), Commands.waitSeconds(0.2)))),
        CommandUtils.annotate(
            "twist to pickup",
            Commands.deadline(
                CommandUtils.selectByMode(
                    manipulator.grabber.intakeAlgae(), Commands.waitSeconds(0.5)),
                swerveDrive.drivePreciselyTo(pickupPose)
            )));
  }

  /**
   * Returns a command that intakes coral and then raises the elevator.
   *
   * @return Command for intake then elevator ready
   */
  private Command intakeThenRaiseElevator() {
    return Commands.sequence(
        pieceCombos.intakeCoral(),
        elevator.ready()
    );
  }

  /**
   * Returns a command sequence to place coral at a specified pole and level.
   *
   * @param position The target CoralPosition (pole and level)
   * @return Command for placing coral
   */
  public Command placeCoral(CoralPosition position) {
    if (position.level == 1) {
      throw new IllegalArgumentException("Cannot place coral at level 1 during autonomous");
    }

    Pose2d placePose = ReefPositioning.getCoralPlacePose(position.pole);

    return Commands.sequence(
        // In case the robot got to the coral pole before it was done intaking,
        // continue intaking until a coral is in the manipulator and clear of
        // the funnel. The timeout is to prevent the robot from getting
        // stuck if it failed to pick up coral.
        CommandUtils.onlyIf(
                () -> !manipulator.grabber.hasCoral() || !manipulator.grabber.isCoralClear(),
                CommandUtils.annotate("intake coral", pieceCombos.intakeCoral()))
            .withTimeout(1.0),
        // Continue intaking in case the coral hasn't reached the "has
        // coral" beam break but has gotten to "coral clear"
        CommandUtils.onlyIf(
            () ->
                RobotBase.isReal()
                    && !manipulator.grabber.hasCoral()
                    && !manipulator.grabber.isCoralClear(),
            CommandUtils.annotate("intake coral because coral clear", pieceCombos.intakeCoral())),
        // If the robot has coral, align to the place pose, raise the
        // elevator, and drop it. Otherwise, end the command, skipping this
        // pole.
        CommandUtils.onlyIf(
            () ->
                RobotBase.isSimulation()
                    || (manipulator.grabber.hasCoral() && manipulator.grabber.isCoralClear()),
            Commands.sequence(
                // Move the elevator and manipulator to the correct placing
                // position, aligning at the same time.
                CommandUtils.annotate(
                    "position mechanisms for place",
                    Commands.deadline(
                        CommandUtils.selectByMode(
                            pieceCombos.coral(position.level), Commands.waitSeconds(0.25)),
                        manipulator.grabber.repositionCoral())),
                // Finish aligning while holding the elevator and
                // manipulator in the same place.
                CommandUtils.annotate(
                    "align",
                    swerveDrive
                        .drivePreciselyTo(placePose)
                        .until(
                            () ->
                                swerveDrive.isWithinToleranceOf(
                                    placePose, Inches.of(0.85), Degrees.of(4)))),
                CommandUtils.annotate("reposition coral", manipulator.grabber.repositionCoral()),
                // Drop the coral while keeping the elevator and manipulator
                // in place.
                CommandUtils.annotate(
                    "drop coral",
                    manipulator
                        .grabber
                        .dropCoral()),
                // Stow the pivot
                CommandUtils.annotate(
                    "stow pivot",
                    CommandUtils.selectByMode(
                        manipulator.pivot.stow(), Commands.waitSeconds(0.1))))));
  }

  /**
   * Creates a command to intake coral from a station slot, driving to the appropriate pose and waiting up to the given timeout.
   *
   * @param coralStation The coral station (LEFT/RIGHT)
   * @param slot         The slot index
   * @param waitTime     The maximum time to wait
   * @return Command for intaking coral from the station
   */
  public Command intakeCoral(CoralStation coralStation, int slot, Time waitTime) {
    Pose2d intakePose = StationPositioning.getIntakePose(coralStation, slot);

    return Commands.race(
        swerveDrive.drivePreciselyTo(intakePose).withTimeout(waitTime), pieceCombos.intakeCoral());
  }
}
