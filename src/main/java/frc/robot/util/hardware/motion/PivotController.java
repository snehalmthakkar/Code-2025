package frc.robot.util.hardware.motion;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.telemetry.StatusChecks;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.MANIPULATOR_PIVOT;
import frc.robot.util.hardware.SparkMaxUtil;

/**
 * Uses a Spark Max motor controller with a NEO brushless motor and a Rev
 * Through-Bore absolute encoder to control the angle of a pivoting arm.
 * 
 * @deprecated Do not use after the 2025 season.
 */
public class PivotController extends SubsystemBase {
  private Angle targetPosition = null;
  private double kG = 0.0;
  // Onboard spark max PID controller. Runs at 1kHz
  private SparkClosedLoopController pid;
  // CAN Spark Max motor controller;
  protected SparkMax motor;
  // Built-in relative NEO encoder
  protected RelativeEncoder encoder;
  // Rev absolute through-bore encoder
  protected AbsoluteEncoder absoluteEncoder;

  private Angle minAngle, maxAngle;

  public final Angle tolerance;

  private final boolean reversed;

  private Debouncer debouncer = new Debouncer(0.1);

  public PivotController(
      String name,
      int motorCAN,
      double absolutePositionOffset,
      double kP,
      double kI,
      double kD,
      double kG,
      double gearing,
      Angle minAngle,
      Angle maxAngle,
      Angle tolerance,
      boolean reversed) {

    setName(name);

    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motor = new SparkMax(motorCAN, MotorType.kBrushless);
    encoder = motor.getEncoder();
    pid = motor.getClosedLoopController();

    absoluteEncoder = motor.getAbsoluteEncoder();

    motorConfig.absoluteEncoder.zeroOffset(absolutePositionOffset);

    this.kG = kG;
    this.minAngle = minAngle;
    this.maxAngle = maxAngle;
    this.tolerance = tolerance;

    this.reversed = reversed;

    SparkMaxUtil.configure(motorConfig, false, IdleMode.kBrake);
    SparkMaxUtil.configureEncoder(motorConfig, 1.0 / gearing);
    SparkMaxUtil.configurePID(
        motorConfig, kP, kI, kD, 0.0, minAngle.in(Rotations), maxAngle.in(Rotations), false);
    SparkMaxUtil.saveAndLog(this, motor, motorConfig);

    StatusChecks.Category statusChecks = StatusChecks.under(this);
    statusChecks.add("absoluteEncoderUpdated", () -> absoluteEncoder.getPosition() != 0.0);
    statusChecks.add("motor", motor);

    Logger.logBoolean(this.getName() + "/doneMoving", this::doneMoving);
    Logger.logBoolean(this.getName() + "/safety/forward", this::triggeredForwardSafety);
    Logger.logBoolean(this.getName() + "/safety/reverse", this::triggeredReverseSafety);
    Logger.logNumber(this.getName() + "/duty/applied", () -> motor.getAppliedOutput());
    Logger.logNumber(this.getName() + "/duty/pid", () -> motor.get());

    Logger.logNumber(this.getName() + "/angle/target", () -> targetPosition == null ? 0 : targetPosition.in(Rotations));
    Logger.logNumber(this.getName() + "/angle/relative", () -> getPosition().in(Rotations));
    Logger.logNumber(this.getName() + "/angle/absolute", () -> getAbsolutePosition().in(Rotations));
    Logger.logNumber(this.getName() + "/angle/raw", () -> getRawAbsolutePosition().in(Rotations));
    Logger.logNumber(this.getName() + "/angle/min", () -> minAngle.in(Rotations));
    Logger.logNumber(this.getName() + "/angle/max", () -> maxAngle.in(Rotations));
  }

  public Angle getPosition() {
    return Rotations.of(encoder.getPosition());
  }

  public Command pivotTo(Supplier<Angle> targetSupplier, Angle tolerance) {
    return startEnd(
      () -> pivotTowards(targetSupplier.get()),
      () -> {
        if (doneMoving(tolerance)) {
          targetPosition = targetSupplier.get();
        } else {
          targetPosition = getPosition();
        }

        pivotTowards(targetPosition);
      }
    ).until(() -> doneMoving(tolerance));
  }

  public Command pivotTo(Supplier<Angle> angleSupplier) {
    return pivotTo(angleSupplier, tolerance);
  }

  private void pivotTowards(Angle requestedAngle) {
    if (requestedAngle == null) return; // If we havent set a target angle yet, do nothing
    targetPosition = clampAngle(requestedAngle);

    // Set onboard PID controller to follow

    if (canMoveInDirection(targetPosition.minus(getPosition()).in(Rotations))) {
      pid.setReference(
          targetPosition.in(Rotations),
          ControlType.kPosition,
          ClosedLoopSlot.kSlot0,
          calculateKG(getAbsolutePosition()));
    } else {
      motor.stopMotor();
    }
  }

  private Angle clampAngle(Angle angle) {
    if (angle.gt(maxAngle)) return maxAngle;
    if (angle.lt(minAngle)) return minAngle;
    return angle;
  }

  public Command move(double speed) {
    return new Command() {
      @Override
      public void initialize() {
        targetPosition = null;
      }

      @Override
      public void execute() {
        if (canMoveInDirection(speed)) {
          motor.setVoltage(12.0 * speed);
        } else {
          motor.stopMotor();
        }
      }

      @Override
      public void end(boolean interrupted) {
        motor.stopMotor();
        targetPosition = getPosition();
      }
    };
  }

  public Command up() {
    return move(0.1);
  }

  public Command down() {
    return move(-0.1);
  }

  public double calculateKG(Angle currentAngle) {
    return kG * Math.cos(MANIPULATOR_PIVOT.CENTER_OF_MASS_OFFSET.in(Radians) + currentAngle.in(Radians));
  }

  private Angle wrapAngle(Angle angle) {
    double reverseMultiplier = reversed ? -1 : 1;

    // map from 0 - 1 rotations to -0.5 to 0.5 rotations, where 0 is straight
    // out
    double absoluteAngle = angle.in(Rotations) * reverseMultiplier; // rotations

    // keeps the range between 0 and 1
    if (absoluteAngle < 0) absoluteAngle++;
    absoluteAngle %= 1.0;

    // wrap at 0.5 rotations
    if (absoluteAngle > 0.5) {
      absoluteAngle -= 1;
    }

    return Rotations.of(absoluteAngle);
  }

  private Angle getRawAbsolutePosition() {
    return Rotations.of(absoluteEncoder.getPosition());
  }

  private Angle getAbsolutePosition() {
    return wrapAngle(Rotations.of(absoluteEncoder.getPosition()));
  }

  public boolean canMoveInDirection(double velocity) {
    if (velocity > 0.0) return getAbsolutePosition().lt(maxAngle);
    if (velocity < 0.0) return getAbsolutePosition().gt(minAngle);
    return true;
  }

  private boolean triggeredForwardSafety() {
    return getAbsolutePosition().gt(maxAngle) && encoder.getVelocity() > 0.0;
  }

  private boolean triggeredReverseSafety() {
    return getAbsolutePosition().lt(minAngle) && encoder.getVelocity() < 0.0;
  }

  public boolean doneMoving(Angle tolerance) {
    if (targetPosition == null) return true;
    return debouncer.calculate(
        getPosition().minus(targetPosition).abs(Radians) < tolerance.in(Radians));
  }

  private boolean doneMoving() {
    return doneMoving(tolerance);
  }

  public void setMinMaxAngle(Angle minAngle, Angle maxAngle) {
    this.minAngle = minAngle;
    this.maxAngle = maxAngle;
  }

  @Override
  public void periodic() {
    if (RobotState.isDisabled()) {
      targetPosition = getPosition();
    }

    pivotTowards(targetPosition);

    encoder.setPosition(getAbsolutePosition().in(Rotations));

    if (triggeredForwardSafety() || triggeredReverseSafety()) {
      motor.stopMotor();
    }
  }
}
