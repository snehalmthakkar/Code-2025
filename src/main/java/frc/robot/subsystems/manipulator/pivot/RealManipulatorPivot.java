package frc.robot.subsystems.manipulator.pivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants.CAN;
import frc.robot.constants.Constants.DIO;
import frc.robot.constants.Constants.MANIPULATOR_PIVOT;
import frc.robot.util.hardware.motion.PivotController;

@SuppressWarnings("deprecation")
public class RealManipulatorPivot extends PivotController implements ManipulatorPivot {
  public RealManipulatorPivot() {
    super(
        "Manipulator Pivot",
        CAN.MANIPULATOR_PIVOT,
        DIO.MANIPULATOR_ENCODER,
        MANIPULATOR_PIVOT.ABSOLUTE_POSITION_OFFSET.in(Rotations),
        MANIPULATOR_PIVOT.PROFILE.kP,
        MANIPULATOR_PIVOT.PROFILE.kI,
        MANIPULATOR_PIVOT.PROFILE.kD,
        MANIPULATOR_PIVOT.PROFILE.kS,
        MANIPULATOR_PIVOT.GEARING,
        MANIPULATOR_PIVOT.MIN_ANGLE,
        MANIPULATOR_PIVOT.MAX_ANGLE,
        MANIPULATOR_PIVOT.TOLERANCE,
        MANIPULATOR_PIVOT.INVERTED);
  }

  @Override
  public Command coralIntake() {
    return pivotTo(() -> MANIPULATOR_PIVOT.CORAL.INTAKE_ANGLE);
  }

  @Override
  public Command coralL1() {
    return pivotTo(() -> MANIPULATOR_PIVOT.CORAL.L1_ANGLE);
  }

  @Override
  public Command coralL23() {
    return pivotTo(() -> MANIPULATOR_PIVOT.CORAL.L23_ANGLE);
  }

  @Override
  public Command coralL4() {
    return pivotTo(() -> MANIPULATOR_PIVOT.CORAL.L4_ANGLE);
  }

  @Override
  public Command algaeReef() {
    return pivotTo(() -> MANIPULATOR_PIVOT.ALGAE.REEF_ANGLE);
  }

  @Override
  public Command algaeBargeSetup() {
    return pivotTo(() -> MANIPULATOR_PIVOT.ALGAE.BARGE.AIM_ANGLE)
        .until(() -> getPosition().gt(MANIPULATOR_PIVOT.MAX_ANGLE.minus(Degrees.of(1))));
  }

  @Override
  public Command algaeBargeShoot() {
    return pivotTo(() -> MANIPULATOR_PIVOT.ALGAE.BARGE.END_ANGLE);
  }

  @Override
  public Command algaeProcessor() {
    return pivotTo(() -> MANIPULATOR_PIVOT.ALGAE.PROCESSOR_ANGLE);
  }

  @Override
  public Command algaeGround() {
    return pivotTo(() -> MANIPULATOR_PIVOT.ALGAE.GROUND_ANGLE);
  }

  @Override
  public Command stow() {
    return pivotTo(() -> MANIPULATOR_PIVOT.STOW_ANGLE);
  }

  @Override
  public Command safe(Angle tolerance) {
    return pivotTo(() -> MANIPULATOR_PIVOT.SAFE_ANGLE, tolerance)
        .until(
            () ->
                getPosition().lt(MANIPULATOR_PIVOT.SAFE_MAX_ANGLE)
                    && getPosition().gt(MANIPULATOR_PIVOT.SAFE_MIN_ANGLE));
  }

  @Override
  public boolean inRange(Angle angle) {
    return getPosition().minus(angle).abs(Rotations)
        < MANIPULATOR_PIVOT.TOLERANCE.in(Rotations);
  }

  @Override
  public Command safe() {
    return safe(MANIPULATOR_PIVOT.SAFE_TOLERANCE);
  }

  public Command pidTestMin() {
    return pivotTo(() -> MANIPULATOR_PIVOT.PID_MIN_ANGLE);
  }

  public Command pidTestMid() {
    return pivotTo(() -> MANIPULATOR_PIVOT.PID_MID_ANGLE);
  }

  public Command pidTestMax() {
    return pivotTo(() -> MANIPULATOR_PIVOT.PID_MAX_ANGLE);
  }

  public Command calibrate() {
    SysIdRoutine calibrationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(Volts.per(Second).of(4.0), Volts.of(6.0), Seconds.of(1.5)),
            new SysIdRoutine.Mechanism(
                voltage -> {
                  if (!canMoveInDirection(voltage.in(Volts))) {
                    DriverStation.reportError("Reached limit switch", false);

                    return;
                  }

                  motor.setVoltage(voltage);
                },
                log ->
                    log.motor("manipulator-pivot")
                        .voltage(Volts.of(motor.getAppliedOutput() * motor.getBusVoltage()))
                        .angularPosition(getPosition())
                        .angularVelocity(RotationsPerSecond.of(motor.getEncoder().getVelocity())),
                this));

    return Commands.sequence(
        Commands.waitSeconds(1.0),
        calibrationRoutine.quasistatic(SysIdRoutine.Direction.kForward),
        Commands.runOnce(motor::stopMotor),
        Commands.waitSeconds(1.0),
        calibrationRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
        Commands.runOnce(motor::stopMotor),
        Commands.waitSeconds(1.0),
        calibrationRoutine.dynamic(SysIdRoutine.Direction.kForward),
        Commands.runOnce(motor::stopMotor),
        Commands.waitSeconds(1.0),
        calibrationRoutine.dynamic(SysIdRoutine.Direction.kReverse),
        Commands.runOnce(motor::stopMotor),
        Commands.waitSeconds(1.0));
  }
}
