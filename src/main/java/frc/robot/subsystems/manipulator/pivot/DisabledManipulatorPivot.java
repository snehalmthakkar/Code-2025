package frc.robot.subsystems.manipulator.pivot;

import static edu.wpi.first.units.Units.Radians;

import com.team6962.lib.utils.CommandUtils;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class DisabledManipulatorPivot implements ManipulatorPivot {
  @Override
  public Angle getPosition() {
    return Radians.of(0);
  }

  @Override
  public Command pivotTo(Supplier<Angle> angleSupplier, Angle tolerance) {
    return CommandUtils.noneWithRequirements(this);
  }

  @Override
  public Command pivotTo(Supplier<Angle> angleSupplier) {
    return CommandUtils.noneWithRequirements(this);
  }

  @Override
  public Command coralIntake() {
    return CommandUtils.noneWithRequirements(this);
  }

  @Override
  public Command coralL1() {
    return CommandUtils.noneWithRequirements(this);
  }

  @Override
  public Command coralL23() {
    return CommandUtils.noneWithRequirements(this);
  }

  @Override
  public Command coralL4() {
    return CommandUtils.noneWithRequirements(this);
  }

  @Override
  public Command algaeReef() {
    return CommandUtils.noneWithRequirements(this);
  }

  @Override
  public Command algaeBargeSetup() {
    return CommandUtils.noneWithRequirements(this);
  }

  @Override
  public Command algaeBargeShoot() {
    return CommandUtils.noneWithRequirements(this);
  }

  @Override
  public Command algaeProcessor() {
    return CommandUtils.noneWithRequirements(this);
  }

  @Override
  public Command algaeGround() {
    return CommandUtils.noneWithRequirements(this);
  }

  @Override
  public Command stow() {
    return CommandUtils.noneWithRequirements(this);
  }

  @Override
  public Command safe(Angle tolerance) {
    return CommandUtils.noneWithRequirements(this);
  }

  @Override
  public Command safe() {
    return CommandUtils.noneWithRequirements(this);
  }

  @Override
  public Command up() {
    return CommandUtils.noneWithRequirements(this);
  }

  @Override
  public Command down() {
    return CommandUtils.noneWithRequirements(this);
  }

  @Override
  public void setMinMaxAngle(Angle min, Angle max) {
  }

  @Override
  public boolean inRange(Angle angle) {
    return true;
  }
}
