package frc.robot.subsystems.manipulator.grabber;

import static edu.wpi.first.units.Units.Seconds;

import com.team6962.lib.utils.CommandUtils;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants.MANIPULATOR;

public class SimGrabber extends Grabber {
  private boolean hasCoral = true;

  private boolean hasAlgae = false;

  private final Time coralIntakeTime;
  private final Time coralDropTime;

  private final Time algaeIntakeTime;
  private final Time algaeDropTime;

  public SimGrabber(
      Time coralIntakeTime,
      Time coralDropTime,
      Time algaeIntakeTime,
      Time algaeDropTime,
      Time stopTime,
      Time algaeGripTime) {
    this.coralIntakeTime = coralIntakeTime;
    this.coralDropTime = coralDropTime;

    this.algaeIntakeTime = algaeIntakeTime;
    this.algaeDropTime = algaeDropTime;
  }

  @Override
  public boolean isAlgaeFullyIntaked() {
      return hasAlgae;
  }

  public static SimGrabber simulated() {
    return new SimGrabber(
        Seconds.of(0.25),
        Seconds.of(0.25),
        Seconds.of(0.25),
        Seconds.of(0.25),
        MANIPULATOR.ALGAE_GRIP_CHECK_TIME,
        Seconds.of(0));
  }

  public static SimGrabber disabled() {
    return new SimGrabber(
        Seconds.of(0), Seconds.of(0), Seconds.of(0), Seconds.of(0), Seconds.of(0), Seconds.of(0));
  }

  private Command waitTimeCommand(Time time) {
    Command command =
        time.isNear(Seconds.of(0), Seconds.of(0.01)) ? Commands.none() : Commands.waitTime(time);

    command.addRequirements(this);
    return command;
  }

  @Override
  public boolean hasCoral() {
    return hasCoral;
  }

  public void expectCoral(boolean value) {
    hasCoral = value;
  }

  @Override
  public Command intakeCoral() {
    return waitTimeCommand(coralIntakeTime).andThen(runOnce(() -> expectCoral(true)));
  }

  @Override
  public Command dropCoral() {
    return waitTimeCommand(coralDropTime).andThen(runOnce(() -> expectCoral(false)));
  }

  @Override
  public Command adjustCoral() {
    return waitTimeCommand(Seconds.of(0.0));
  }

  @Override
  public boolean hasAlgae() {
    return hasAlgae;
  }

  public void expectAlgae(boolean hasAlgae) {
    this.hasAlgae = hasAlgae;
  }

  @Override
  public Command intakeAlgae() {
    return waitTimeCommand(algaeIntakeTime).andThen(runOnce(() -> expectAlgae(true)));
  }

  @Override
  public Command dropAlgae() {
    return waitTimeCommand(algaeDropTime).andThen(runOnce(() -> expectAlgae(false)));
  }

  @Override
  public Command repositionCoral() {
    return CommandUtils.noneWithRequirements(this);
  }
}
