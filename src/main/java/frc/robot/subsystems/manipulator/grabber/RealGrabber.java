package frc.robot.subsystems.manipulator.grabber;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.telemetry.StatusChecks;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants.CAN;
import frc.robot.constants.Constants.DIO;
import frc.robot.constants.Constants.MANIPULATOR;
import frc.robot.util.hardware.SparkMaxUtil;

public class RealGrabber extends Grabber {
  private final SparkMax motor;
  private final DigitalInput detectSensor;
  private final DigitalInput clearSensor;
  private final AlgaeSensor algaeSensor;

  private boolean hasCoral = false;
  private boolean coralClear = false;
  private boolean running = false;

  public RealGrabber() {
    motor = new SparkMax(CAN.MANIPULATOR_GRABBER, MotorType.kBrushless);

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
    algaeSensor = new AlgaeSensor(CAN.ALGAE_SENSOR, "");

    StatusChecks.under(this).add("motor", motor);
  }

  private Command run(double speed) {
    return startEnd(
      () -> {
        running = true;
        motor.set(speed);
      },
      () -> {
        running = false;
        motor.set(getStoppedSpeed());
      }
    );
  }

  private double getStoppedSpeed() {
    return hasAlgae() ? MANIPULATOR.ALGAE_HOLD_SPEED : 0.0;
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
      run(MANIPULATOR.CORAL_IN_SPEED).until(this::hasCoral),
      run(MANIPULATOR.CORAL_SLOW_IN_SPEED).until(this::isCoralClear).onlyIf(() -> !hasCoral || !coralClear)
    );
  }

  @Override
  public Command dropCoral() {
    return run(MANIPULATOR.CORAL_OUT_SPEED)
        .withDeadline(
            Commands.sequence(
                Commands.waitUntil(this::hasCoral), Commands.waitUntil(() -> !hasCoral())));
  }

  @Override
  public Command adjustCoral() {
    return run(MANIPULATOR.CORAL_ADJUST_SPEED);
  }

  @Override
  public Command repositionCoral() {
    return Commands.sequence(
        run(MANIPULATOR.CORAL_ADJUST_SPEED).until(() -> !isCoralClear()),
        run(MANIPULATOR.CORAL_REPOSITION_SPEED).until(() -> isCoralClear())
      );
  }

  @Override
  public Command intakeAlgae() {
    return run(MANIPULATOR.ALGAE_IN_SPEED)
        .until(this::isAlgaeFullyIntaked);
  }

  @Override
  public Command dropAlgae() {
    return Commands.either(
      Commands.none(),
      run(MANIPULATOR.ALGAE_OUT_SPEED),
      () -> hasCoral()
    );
  }

  @Override
  public Command forwards() {
    return run(MANIPULATOR.BASE_SPEED);
  }

  @Override
  public Command backwards() {
    return run(-MANIPULATOR.BASE_SPEED);
  }

  @Override
  public boolean hasAlgae() {
    return algaeSensor.hasAlgae() && !hasCoral() && isCoralClear();
  }

  @Override
  public boolean isAlgaeFullyIntaked() {
    return algaeSensor.isAlgaeFullyIntaked() && !hasCoral() && isCoralClear();
  }

  @Override
  public void periodic() {
    super.periodic();

    hasCoral = !detectSensor.get();
    coralClear = clearSensor.get();

    if (!running) {
      motor.set(getStoppedSpeed());
    }
  }
}
