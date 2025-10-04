package frc.robot.commands;

import com.team6962.lib.utils.CommandUtils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.constants.Constants.ELEVATOR;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.manipulator.Manipulator;

public class PieceCombos {
  Elevator elevator;
  Manipulator manipulator;
  SafeSubsystems safeSubsystems;

  public PieceCombos(Elevator elevator, Manipulator manipulator, SafeSubsystems safeSubsystems) {
    this.elevator = elevator;
    this.manipulator = manipulator;
    this.safeSubsystems = safeSubsystems;
  }

  public Command intakeCoral() {
    return safeSubsystems
        .safeMoveCommand(
            elevator.coralIntake(), manipulator.intakeCoral(), ELEVATOR.CORAL.INTAKE_HEIGHT)
        .withName("CORAL INTAKE");
  }

  public Command coral(int level) {
    return switch (level) {
      case 1 -> coralL1();
      case 2 -> coralL2();
      case 3 -> coralL3();
      case 4 -> coralL4();
      default -> CommandUtils.noneWithRequirements(elevator, manipulator);
    };
  }

  public Command coralL1() {
    return safeSubsystems
        .safeMoveCommand(elevator.coralL1(), manipulator.placeCoralL1(), ELEVATOR.CORAL.L1_HEIGHT)
        .withName("CORAL L1");
  }

  public Command coralL2() {
    return safeSubsystems
        .safeMoveCommand(elevator.coralL2(), manipulator.placeCoralL23(), ELEVATOR.CORAL.L2_HEIGHT)
        .withName("CORAL L2");
  }

  public Command coralL3() {
    return safeSubsystems
        .safeMoveCommand(elevator.coralL3(), manipulator.placeCoralL23(), ELEVATOR.CORAL.L3_HEIGHT)
        .withName("CORAL L3");
  }

  public Command readyL3() {
    return safeSubsystems
        .safeMoveCommand(
            elevator.ready(), manipulator.stow(), ELEVATOR.AUTO.READY_HEIGHT)
        .withName("READY L3");
  }

  public Command readyL2() {
    return safeSubsystems
        .safeMoveCommand(
            elevator.coralL2(), manipulator.stow(), ELEVATOR.AUTO.READY_HEIGHT)
        .withName("READY L3");
  }

  public Command coralL4() {
    return safeSubsystems
        .safeMoveCommand(elevator.coralL4(), manipulator.placeCoralL4(), ELEVATOR.CORAL.L4_HEIGHT)
        .withName("CORAL L4");
  }

  public Command algae(int level) {
    return switch (level) {
      case 2 -> algaeL2();
      case 3 -> algaeL3();
      default -> CommandUtils.noneWithRequirements(elevator, manipulator);
    };
  }

  public Command algaeL2() {
    return safeSubsystems
        .safeMoveCommand(
            elevator.algaeL2(), manipulator.pivot.algaeReef(), ELEVATOR.ALGAE.L2_HEIGHT)
        .withName("ALGAE L2");
  }

  public Command algaeL3() {
    return safeSubsystems
        .safeMoveCommand(
            elevator.algaeL3(), manipulator.pivot.algaeReef(), ELEVATOR.ALGAE.L3_HEIGHT)
        .withName("ALGAE L3");
  }

  public Command algaeBargeSetup() {
    return safeSubsystems
        .safeMoveCommand(
            elevator.algaeBarge(), manipulator.pivot.algaeBargeSetup(), ELEVATOR.ALGAE.BARGE_HEIGHT)
        .withName("BARGE SETUP");
  }

  public Command algaeBargeShoot() {
    return Commands.sequence(
            manipulator.pivot.algaeBargeSetup(),
            manipulator.pivot.algaeBargeShoot().deadlineFor(Commands.sequence(
              Commands.waitSeconds(0.15),
              manipulator.grabber.dropAlgae())
            ))
        .onlyIf(() -> elevator.isNear(ELEVATOR.ALGAE.BARGE_HEIGHT));
  }

  public Command stow() {
    return manipulator
        .pivot
        .safe()
        .andThen(elevator.stow())
        .andThen(manipulator.stow())
        .withName("STOW");
  }

  public Command intakeAlgaeOrShootCoral() {
    return new ConditionalCommand(
        manipulator.grabber.dropCoral(),
        manipulator.grabber.intakeAlgae(),
        () -> manipulator.grabber.hasCoral());
  }
}
