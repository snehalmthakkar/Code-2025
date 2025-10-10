package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.ELEVATOR;
import frc.robot.constants.Constants.MANIPULATOR_PIVOT;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.manipulator.Manipulator;

/**
 * SafeSubsystems coordinates the safe operation of subsystems that have mechanical dependencies,
 * such as the elevator and manipulator. It ensures that motions are performed within safe bounds
 * and provides helper commands for safe sequencing and parallelization.
 */
public class SafeSubsystems extends SubsystemBase {
  Manipulator manipulator;
  Elevator elevator;

  /**
   * Constructs a SafeSubsystems instance with the provided elevator and manipulator subsystems.
   *
   * @param elevator The elevator subsystem
   * @param manipulator The manipulator subsystem
   */
  public SafeSubsystems(Elevator elevator, Manipulator manipulator) {
    this.elevator = elevator;
    this.manipulator = manipulator;
  }

  /**
   * Calculates the minimum safe angle for the manipulator pivot given the elevator height.
   * Falls back to a default safe angle if the lookup fails.
   *
   * @param elevatorHeight The height of the elevator
   * @return The minimum safe angle for the manipulator pivot at the given elevator height
   */
  public static Angle calcSafeMinAngle(Distance elevatorHeight) {
    Angle returnAngle = MANIPULATOR_PIVOT.MIN_ANGLES.ceilingEntry(elevatorHeight).getValue();
    if (returnAngle == null) {
      return MANIPULATOR_PIVOT.SAFE_ANGLE;
    }

    return returnAngle;
  }

  /**
   * Calculates the maximum safe angle for the manipulator pivot given the elevator height.
   * Uses a special intake angle if the elevator is below a threshold,
   * otherwise returns the mapped max angle or falls back to a default.
   *
   * @param elevatorHeight The height of the elevator
   * @return The maximum safe angle for the manipulator pivot at the given elevator height
   */
  public static Angle calcSafeMaxAngle(Distance elevatorHeight) {
    if (elevatorHeight.lt(ELEVATOR.CORAL.INTAKE_HEIGHT.plus(Inches.of(4)))) {
      return MANIPULATOR_PIVOT.CORAL.INTAKE_ANGLE;
    }

    Angle returnAngle = MANIPULATOR_PIVOT.MAX_ANGLES.ceilingEntry(elevatorHeight).getValue();
    if (returnAngle == null) {
      return MANIPULATOR_PIVOT.SAFE_ANGLE;
    }
    return returnAngle;
  }

  /**
   * Creates a command sequence that ensures manipulator safety before moving the elevator,
   * followed by the desired elevator and manipulator commands. If the elevator is already at (or close to)
   * the target height, the manipulator safety command is skipped.
   *
   * @param elevatorCommand The command to move the elevator
   * @param manipulatorCommand The command to move the manipulator
   * @param targetHeight The target elevator height
   * @return A command sequence that safely moves the elevator and manipulator
   */
  public Command safeMoveCommand(
      Command elevatorCommand, Command manipulatorCommand, Distance targetHeight) {
    Command safeMoveCommand =
        new ConditionalCommand(
            Commands.none(),
            manipulator.pivot.safe(),
            () ->
                (targetHeight.minus(elevator.getPosition()).abs(Inches)
                    < ELEVATOR.TOLERANCE.abs(Inches)));
    return Commands.sequence(safeMoveCommand, elevatorCommand, manipulatorCommand);
  }

  /**
   * Creates a parallel command for elevator and manipulator movement, but enforces a safety command
   * for the manipulator pivot if safeties are enabled in the configuration. Otherwise, both commands run in parallel.
   *
   * @param elevatorCommand The command to move the elevator
   * @param manipulatorCommand The command to move the manipulator
   * @return A command sequence that respects safety constraints if enabled
   */
  public Command parallelSafeCommand(Command elevatorCommand, Command manipulatorCommand) {
    if (Constants.SAFETIES_ENABLED) {
      return Commands.sequence(
          manipulator.pivot.safe(), Commands.parallel(elevatorCommand, manipulatorCommand));
    }

    return Commands.parallel(elevatorCommand, manipulatorCommand);
  }

  /**
   * Periodic update method called by the scheduler. When safeties are enabled, updates the manipulator
   * pivot's minimum and maximum angles based on the current elevator height.
   */
  @Override
  public void periodic() {
    if (Constants.SAFETIES_ENABLED) {
      Distance elevatorHeight = elevator.getPosition();
      Angle minAngle = calcSafeMinAngle(elevatorHeight);
      Angle maxAngle = calcSafeMaxAngle(elevatorHeight);

      manipulator.pivot.setMinMaxAngle(minAngle, maxAngle);
    }
  }
}
