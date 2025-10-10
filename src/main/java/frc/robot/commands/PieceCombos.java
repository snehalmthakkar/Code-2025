package frc.robot.commands;

import com.team6962.lib.utils.CommandUtils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.constants.Constants.ELEVATOR;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.manipulator.Manipulator;

/**
 * PieceCombos provides composite command generators for scoring or intaking game pieces
 * (such as "coral" or "algae") using the Elevator and Manipulator subsystems in a safe, 
 * coordinated fashion. Each method returns a WPILib Command representing an action sequence 
 * or state for the robot.
 */
public class PieceCombos {
  Elevator elevator;
  Manipulator manipulator;
  SafeSubsystems safeSubsystems;

  /**
   * Constructs a PieceCombos helper for coordinated manipulator/elevator actions.
   * 
   * @param elevator The Elevator subsystem used for vertical movement.
   * @param manipulator The Manipulator subsystem for handling game pieces.
   * @param safeSubsystems Helper for safe multi-subsystem moves.
   */
  public PieceCombos(Elevator elevator, Manipulator manipulator, SafeSubsystems safeSubsystems) {
    this.elevator = elevator;
    this.manipulator = manipulator;
    this.safeSubsystems = safeSubsystems;
  }

  /**
   * Creates a command to intake a "coral" piece, safely moving the elevator to the intake height
   * and activating the manipulator's coral intake function.
   * 
   * @return A Command that performs coral intake.
   */
  public Command intakeCoral() {
    return safeSubsystems
        .safeMoveCommand(
            elevator.coralIntake(), 
            manipulator.intakeCoral(), 
            ELEVATOR.CORAL.INTAKE_HEIGHT)
        .withName("CORAL INTAKE");
  }

  /**
   * Returns a command to place a coral at a specified level (1-4).
   * 
   * @param level The field level to score coral (1 = lowest, 4 = highest).
   * @return The corresponding coral placement command, or a no-op if level is invalid.
   */
  public Command coral(int level) {
    return switch (level) {
      case 1 -> coralL1();
      case 2 -> coralL2();
      case 3 -> coralL3();
      case 4 -> coralL4();
      default -> CommandUtils.noneWithRequirements(elevator, manipulator);
    };
  }

  /**
   * Returns a command to place coral at Level 1 (lowest).
   * 
   * @return Command for coral Level 1 placement.
   */
  public Command coralL1() {
    return safeSubsystems
        .safeMoveCommand(
            elevator.coralL1(), 
            manipulator.placeCoralL1(), 
            ELEVATOR.CORAL.L1_HEIGHT)
        .withName("CORAL L1");
  }

  /**
   * Returns a command to place coral at Level 2.
   * 
   * @return Command for coral Level 2 placement.
   */
  public Command coralL2() {
    return safeSubsystems
        .safeMoveCommand(
            elevator.coralL2(), 
            manipulator.placeCoralL23(), 
            ELEVATOR.CORAL.L2_HEIGHT)
        .withName("CORAL L2");
  }

  /**
   * Returns a command to place coral at Level 3.
   * 
   * @return Command for coral Level 3 placement.
   */
  public Command coralL3() {
    return safeSubsystems
        .safeMoveCommand(
            elevator.coralL3(), 
            manipulator.placeCoralL23(), 
            ELEVATOR.CORAL.L3_HEIGHT)
        .withName("CORAL L3");
  }

  /**
   * Returns a command to move to a "ready" position for Level 3 scoring.
   * 
   * @return Command for readying Level 3.
   */
  public Command readyL3() {
    return safeSubsystems
        .safeMoveCommand(
            elevator.ready(), 
            manipulator.stow(), 
            ELEVATOR.AUTO.READY_HEIGHT)
        .withName("READY L3");
  }

  /**
   * Returns a command to move to a "ready" position using Level 2 elevator height.
   * 
   * @return Command for readying Level 2 (but named L3 for compatibility).
   */
  public Command readyL2() {
    return safeSubsystems
        .safeMoveCommand(
            elevator.coralL2(), 
            manipulator.stow(), 
            ELEVATOR.AUTO.READY_HEIGHT)
        .withName("READY L3");
  }

  /**
   * Returns a command to place coral at Level 4 (highest).
   * 
   * @return Command for coral Level 4 placement.
   */
  public Command coralL4() {
    return safeSubsystems
        .safeMoveCommand(
            elevator.coralL4(), 
            manipulator.placeCoralL4(), 
            ELEVATOR.CORAL.L4_HEIGHT)
        .withName("CORAL L4");
  }

  /**
   * Returns a command to place "algae" at a specified level (2 or 3).
   * 
   * @param level Either 2 or 3, corresponding to field level.
   * @return The corresponding algae placement command, or a no-op if level is invalid.
   */
  public Command algae(int level) {
    return switch (level) {
      case 2 -> algaeL2();
      case 3 -> algaeL3();
      default -> CommandUtils.noneWithRequirements(elevator, manipulator);
    };
  }

  /**
   * Returns a command to place algae at Level 2.
   * 
   * @return Command for algae Level 2 placement.
   */
  public Command algaeL2() {
    return safeSubsystems
        .safeMoveCommand(
            elevator.algaeL2(), 
            manipulator.pivot.algaeReef(), 
            ELEVATOR.ALGAE.L2_HEIGHT)
        .withName("ALGAE L2");
  }

  /**
   * Returns a command to place algae at Level 3.
   * 
   * @return Command for algae Level 3 placement.
   */
  public Command algaeL3() {
    return safeSubsystems
        .safeMoveCommand(
            elevator.algaeL3(), 
            manipulator.pivot.algaeReef(), 
            ELEVATOR.ALGAE.L3_HEIGHT)
        .withName("ALGAE L3");
  }

  /**
   * Returns a command that moves the elevator and manipulator to a setup position for an algae barge.
   * 
   * @return Command for algae barge setup.
   */
  public Command algaeBargeSetup() {
    return safeSubsystems
        .safeMoveCommand(
            elevator.algaeBarge(), 
            manipulator.pivot.algaeBargeSetup(), 
            ELEVATOR.ALGAE.BARGE_HEIGHT)
        .withName("BARGE SETUP");
  }

  /**
   * Returns a command sequence to shoot algae from the barge position.
   * 
   * Only runs if elevator is near the correct barge height.
   * It sequences the manipulator's barge setup, then barge shoot, and coordinates
   * a brief wait before dropping the algae.
   * 
   * @return Command for performing the algae barge shoot.
   */
  public Command algaeBargeShoot() {
    return Commands.sequence(
            manipulator.pivot.algaeBargeSetup(),
            manipulator.pivot.algaeBargeShoot().deadlineFor(
              Commands.sequence(
                Commands.waitSeconds(0.15),
                manipulator.grabber.dropAlgae())
            ))
        .onlyIf(() -> elevator.isNear(ELEVATOR.ALGAE.BARGE_HEIGHT));
  }

  /**
   * Returns a command to stow all mechanisms safely.
   * Sequences manipulator pivot safe, elevator stow, and manipulator stow.
   * 
   * @return Command for stowing all manipulators and elevator.
   */
  public Command stow() {
    return manipulator
        .pivot
        .safe()
        .andThen(elevator.stow())
        .andThen(manipulator.stow())
        .withName("STOW");
  }

  /**
   * Returns a conditional command that either drops a coral (if currently holding one)
   * or starts an intake for algae (if not holding coral).
   * 
   * @return ConditionalCommand for dropping coral or intaking algae.
   */
  public Command intakeAlgaeOrShootCoral() {
    return new ConditionalCommand(
        manipulator.grabber.dropCoral(),
        manipulator.grabber.intakeAlgae(),
        () -> manipulator.grabber.hasCoral());
  }
}
