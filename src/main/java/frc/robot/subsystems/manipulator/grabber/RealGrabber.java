package frc.robot.subsystems.manipulator.grabber;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.telemetry.StatusChecks;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.constants.Constants.CAN;
import frc.robot.constants.Constants.DIO;
import frc.robot.constants.Constants.MANIPULATOR;
import frc.robot.util.hardware.SparkMaxUtil;

public class RealGrabber extends Grabber {

  private final SparkMax motor;
  private final DigitalInput detectSensor;
  private final DigitalInput clearSensor;

  private Debouncer algaeDebouncer = new Debouncer(0.3, DebounceType.kBoth);

  private boolean hasCoral = false;
  private boolean coralClear = false;

  private boolean detectedAlgae = false;

  public RealGrabber() {
    motor = new SparkMax(CAN.MANIPULATOR_GRABBER, MotorType.kBrushless);

    Logger.logBoolean(getName() + "/detectedAlgae", this::detectedAlgae);
    Logger.logBoolean(getName() + "/coralClear", this::isCoralClear);
    Logger.logNumber(getName() + "/vel", () -> motor.getEncoder().getVelocity());
    Logger.logNumber(getName() + "/get", () -> motor.get());
    Logger.logNumber(getName() + "/amps", () -> motor.getOutputCurrent());
    Logger.logNumber(getName() + "/temp", () -> motor.getMotorTemperature());

    SparkMaxConfig config = new SparkMaxConfig();

    SparkMaxUtil.configure(config, false, IdleMode.kBrake);
    SparkMaxUtil.saveAndLog(this, motor, config);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    detectSensor = new DigitalInput(DIO.CORAL_DETECT_BEAM_BREAK);
    clearSensor = new DigitalInput(DIO.CORAL_CLEAR_BEAM_BREAK);

    setDefaultCommand(hold());

    StatusChecks.under(this).add("motor", motor);
  }

  public Command setDutyCycleOnce(double speed) {
    return this.runOnce(() -> motor.set(speed));
  }

  public Command setDutyCycle(double speed) {
    return this.run(() -> motor.set(speed));
  }

  public Command setVoltageOnce(Voltage volts) {
    return this.runOnce(() -> motor.setVoltage(volts));
  }

  public Command setVoltage(Voltage volts) {
    return this.run(() -> motor.setVoltage(volts));
  }

  @Override
  public boolean hasCoral() {
    return hasCoral;
  }

  @Override
  public boolean isCoralClear() {
    return coralClear;
  }

  @Override
  public Command intakeCoral() {
    return Commands.sequence(
        setDutyCycle(MANIPULATOR.CORAL_IN_SPEED).until(this::hasCoral),
        setDutyCycle(MANIPULATOR.CORAL_SLOW_IN_SPEED)
            .withDeadline(
                Commands.sequence(
                    Commands.waitUntil(this::isCoralClear), Commands.waitSeconds(0.0)))
            .onlyIf(() -> !hasCoral || !coralClear),
        stopOnce());
  }

  @Override
  public Command dropCoral() {
    return setDutyCycle(MANIPULATOR.CORAL_OUT_SPEED)
        .withDeadline(
            Commands.sequence(
                Commands.waitUntil(this::hasCoral), Commands.waitUntil(() -> !hasCoral())));
  }

  @Override
  public Command adjustCoral() {
    return setDutyCycle(MANIPULATOR.CORAL_ADJUST_SPEED);
  }

  @Override
  public Command repositionCoral() {
    return Commands.sequence(
        setDutyCycle(MANIPULATOR.CORAL_ADJUST_SPEED).until(() -> !isCoralClear()),
        setDutyCycle(MANIPULATOR.CORAL_REPOSITION_SPEED).until(() -> isCoralClear()),
        stopOnce());
  }

  public boolean detectedAlgae() {
    return detectedAlgae;
  }

  @Override
  public Command intakeAlgae() {
    return setDutyCycle(MANIPULATOR.ALGAE_IN_SPEED)
        .until(this::detectedAlgae)
        .finallyDo(() -> expectAlgae(true));
  }

  @Override
  public Command dropAlgae() {
    return new ConditionalCommand(
        Commands.none(),
        setDutyCycle(MANIPULATOR.ALGAE_OUT_SPEED).finallyDo(() -> expectAlgae(false)),
        () -> hasCoral());
  }

  @Override
  public Command forwards() {
    return setDutyCycle(MANIPULATOR.BASE_SPEED);
  }

  @Override
  public Command backwards() {
    return setDutyCycle(-MANIPULATOR.BASE_SPEED);
  }

  @Override
  public Command holdAlgae() {
    if (!MANIPULATOR.ALGAE_GRIP_CHECK_ENABLED) return stop();
    return Commands.sequence(
        setDutyCycle(MANIPULATOR.ALGAE_GRIP_CHECK_SPEED)
            .until(this::detectedAlgae)
            .withTimeout(MANIPULATOR.ALGAE_GRIP_CHECK_TIME)
            .finallyDo(() -> expectAlgae(detectedAlgae())),
        setDutyCycle(MANIPULATOR.ALGAE_HOLD_SPEED).withTimeout(MANIPULATOR.ALGAE_GRIP_IDLE_TIME));
  }

  // update this for both game pieces
  @Override
  public Command hold() {
    return Commands.either(runOnce(motor::stopMotor), holdAlgae(), () -> hasCoral() || !hasAlgae()).repeatedly();
  }

  public Command stopOnce() {
    return this.runOnce(motor::stopMotor);
  }

  @Override
  public Command stop() {
    return this.run(motor::stopMotor);
  }

  public boolean isStalled() {
    return motor.getOutputCurrent() > 20;
  }

  @Override
  public void periodic() {
    super.periodic();

    detectedAlgae = algaeDebouncer.calculate(isStalled());
    hasCoral = !detectSensor.get();
    coralClear = clearSensor.get();
  }
}
