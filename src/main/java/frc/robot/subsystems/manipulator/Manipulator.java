package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.manipulator.grabber.Grabber;
import frc.robot.subsystems.manipulator.pivot.ManipulatorPivot;

public class Manipulator extends SubsystemBase {
  public final ManipulatorPivot pivot;
  public final Grabber grabber;

  public Manipulator() {
    pivot = ManipulatorPivot.create();
    grabber = Grabber.create();
  }

  public Command placeCoralL1() {
    return pivot.coralL1();
  }

  public Command placeCoralL23() {
    return pivot.coralL23();
  }

  public Command placeCoralL4() {
    return pivot.coralL4();
  }

  public Command intakeCoral() {
    return Commands.sequence(pivot.coralIntake(), pivot.hold()).withDeadline(grabber.intakeCoral());
  }

  public Command pickupGroundAlgae() {
    return pivot.algaeGround().alongWith(grabber.intakeAlgae());
  }

  public Command pickupReefAlgae() {
    return pivot.algaeReef().alongWith(grabber.intakeAlgae());
  }

  public Command dropReefAlgae() {
    return pivot.algaeReef().alongWith(grabber.dropAlgae());
  }

  public Command placeBargeAlgae() {
    return pivot.algaeBargeSetup().andThen(grabber.dropAlgae());
  }

  public Command placeProcessorAlgae() {
    return pivot.algaeProcessor();
  }

  public Command stow() {
    return pivot.stow();
  }

  public Command stop() {
    return pivot.stop().alongWith(grabber.stop());
  }

  public Command test() {
    return Commands.sequence(
        pivot.safe(), /* intakeCoral(), placeCoralL23(),*/
        pickupGroundAlgae(),
        placeProcessorAlgae(),
        pivot.safe());
  }
}
