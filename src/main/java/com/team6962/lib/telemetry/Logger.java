package com.team6962.lib.telemetry;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Arrays;
import java.util.Collection;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.studica.frc.AHRS;
import com.team6962.lib.digitalsensor.DigitalSensor;

import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ENABLED_SYSTEMS;

public class Logger extends SubsystemBase {
  private static NetworkTable table = NetworkTableInstance.getDefault().getTable("Logs");
  private static List<Updatable> updates = new LinkedList<>();
  private static Notifier notifier = new Notifier(Logger::update);
  private static Field2d field2d = new Field2d();
  private static double threadLastPing = Timer.getFPGATimestamp();

  private static record Updatable(String key, Runnable runnable) {
    public void run() {
      try {
        runnable.run();
      } catch (Exception exception) {
        System.out.println("Error logging field: " + key + "\n");
        exception.printStackTrace();
      }
    }
  }

  @Override
  public void periodic() {
    // Logger.log("Logger/notifierAlive", Timer.getFPGATimestamp() - threadLastPing < 0.25);
  }

  public static void start(Time period) {
    System.out.println("Starting periodic");
    notifier.startPeriodic(period.in(Seconds));
    SmartDashboard.putData(field2d);
    SmartDashboard.putData(CommandScheduler.getInstance());
    MechanismLogger.start();

    new Logger();
  }

  private static void update() {
    threadLastPing = Timer.getFPGATimestamp();
    Logger.log("Logger/updateTime", threadLastPing);

    synchronized (updates) {
      for (Updatable update : updates) {
        update.run();
      }
    }
  }

  public static Runnable addUpdate(String key, Runnable runnable) {
    synchronized (updates) {
      updates.add(new Updatable(key, runnable));
    }

    return () -> {
      synchronized (updates) {
        updates.removeIf(update -> update.key.equals(key));
      }
    };
  }

  public static Field2d getField() {
    return field2d;
  }

  public static void logBoolean(String key, Supplier<Boolean> supplier) {
    addUpdate(key, () -> log(key, supplier.get()));
  }

  public static void log(String key, Boolean value) {
    if (value == null) table.getEntry(key).setString("null");
    else table.getEntry(key).setBoolean(value);
  }

  public static void logString(String key, Supplier<String> supplier) {
    addUpdate(key, () -> log(key, supplier.get()));
  }

  public static void log(String key, String value) {
    table.getEntry(key).setString(value == null ? "null" : value);
  }

  public static void logNumber(String key, Supplier<Number> supplier) {
    addUpdate(key, () -> log(key, supplier.get()));
  }

  public static void log(String key, Number value) {
    if (value == null) table.getEntry(key).setValue("null");
    else table.getEntry(key).setNumber(value);
  }

  public static <T extends Unit> void logMeasure(String key, Supplier<Measure<T>> supplier) {
    addUpdate(key, () -> log(key, supplier.get()));
  }

  @SuppressWarnings("unchecked")
  public static <T extends Unit> void log(String key, Measure<T> value) {
    if (value == null) {
      table.getEntry(key).setValue("null");
      return;
    }

    T unit = value.unit();

    if (value instanceof Angle) {
      unit = (T) Rotations;
    } else if (value instanceof Distance) {
      unit = (T) Meters;
    } else if (value instanceof Time) {
      unit = (T) Seconds;
    } else if (value instanceof AngularVelocity) {
      unit = (T) RotationsPerSecond;
    } else if (value instanceof AngularAcceleration) {
      unit = (T) RotationsPerSecondPerSecond;
    } else if (value instanceof LinearVelocity) {
      unit = (T) MetersPerSecond;
    } else if (value instanceof LinearAcceleration) {
      unit = (T) MetersPerSecondPerSecond;
    }

    key += reformatMeasureName(unit.name());

    table.getEntry(key).setValue(value.in(unit));
  }

  private static String reformatMeasureName(String name) {
    return Arrays.stream(name.split(" "))
        .map(str -> str.substring(0, 1).toUpperCase() + str.substring(1).toLowerCase())
        .reduce("", (a, b) -> a + b);
  }

  public static void logBooleanArray(String key, Supplier<boolean[]> supplier) {
    addUpdate(key, () -> log(key, supplier.get()));
  }

  public static void log(String key, boolean[] value) {
    addUpdate(key, () -> table.getEntry(key).setBooleanArray(value));
  }

  public static void logLongArray(String key, Supplier<long[]> supplier) {
    addUpdate(key, () -> log(key, supplier.get()));
  }

  public static void log(String key, long[] value) {
    addUpdate(key, () -> table.getEntry(key).setIntegerArray(value));
  }

  public static void logDoubleArray(String key, Supplier<double[]> supplier) {
    addUpdate(key, () -> log(key, supplier.get()));
  }

  public static void log(String key, double[] value) {
    addUpdate(key, () -> table.getEntry(key).setDoubleArray(value));
  }

  public static void logNumberArray(String key, Supplier<Number[]> supplier) {
    addUpdate(key, () -> log(key, supplier.get()));
  }

  public static void log(String key, Number[] supplier) {
    addUpdate(key, () -> table.getEntry(key).setValue(supplier));
  }

  public static void logTranslation(String key, Supplier<Translation2d> supplier) {
    addUpdate(key, () -> log(key, supplier.get()));
  }

  public static void log(String key, Translation2d value) {
    addUpdate(
        key,
        () -> {
          if (value == null) table.getEntry(key).setString("null");
          else table.getEntry(key).setDoubleArray(new double[] {value.getX(), value.getY()});
        });
  }

  public static void logRotation(String key, Supplier<Rotation2d> supplier) {
    addUpdate(key, () -> log(key, supplier.get()));
  }

  public static void log(String key, Rotation2d value) {
    table.getEntry(key + "Radians").setDouble(value.getRadians());
  }

  public static void logSpeeds(String key, Supplier<ChassisSpeeds> supplier) {
    addUpdate(key, () -> log(key, supplier.get()));
  }

  public static void log(String key, ChassisSpeeds speeds) {
    table
        .getEntry(key)
        .setDoubleArray(
            new double[] {
              speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond
            });
  }

  public static void logPose(String key, Supplier<Pose2d> supplier) {
    addUpdate(key, () -> log(key, supplier.get()));
  }

  public static void log(String key, Pose2d pose) {
    table
        .getEntry(key)
        .setDoubleArray(new double[] {pose.getX(), pose.getY(), pose.getRotation().getRadians()});
  }

  public static void logSwerveModuleState(String key, Supplier<SwerveModuleState> supplier) {
    addUpdate(key, () -> log(key, supplier.get()));
  }

  public static void log(String key, SwerveModuleState state) {
    log(key + "/speedMetersPerSecond", state.speedMetersPerSecond);
    log(key + "/angleRadians", state.angle.getRadians());
  }

  public static void logSwerveModuleStates(String key, Supplier<SwerveModuleState[]> supplier) {
    addUpdate(key, () -> log(key, supplier.get()));
  }

  public static void log(String key, SwerveModuleState[] states) {
    for (int i = 0; i < states.length; i++) {
      log(key + "/" + i, states[i]);
    }
  }

  public static void logSwerveModulePosition(String key, Supplier<SwerveModulePosition> supplier) {
    addUpdate(key, () -> log(key, supplier.get()));
  }

  public static void log(String key, SwerveModulePosition position) {
    log(key + "/distanceMeters", position.distanceMeters);
    log(key + "/angleRadians", position.angle.getRadians());
  }

  public static void logSwerveModulePositions(
      String key, Supplier<SwerveModulePosition[]> supplier) {
    addUpdate(key, () -> log(key, supplier.get()));
  }

  public static void log(String key, SwerveModulePosition[] positions) {
    for (int i = 0; i < positions.length; i++) {
      log(key + "/" + i, positions[i]);
    }
  }

  public static void logDigitalSensor(String key, DigitalSensor sensor) {
    addUpdate(key, () -> log(key, sensor));
  }

  public static void log(String key, DigitalSensor sensor) {
    log(key, sensor.isTriggered());
  }

  public static void logNavX(String key, Supplier<AHRS> supplier) {
    addUpdate(key, () -> log(key, supplier.get()));
  }

  public static void log(String key, AHRS navX) {
    log(key + "/isAltitudeValid", navX.isAltitudeValid());
    log(key + "/isCalibrating", navX.isCalibrating());
    log(key + "/isConnected", navX.isConnected());
    log(key + "/isMagneticDisturbance", navX.isMagneticDisturbance());
    log(key + "/isMagnetometerCalibrated", navX.isMagnetometerCalibrated());
    log(key + "/isMoving", navX.isMoving());
    log(key + "/isRotating", navX.isRotating());
    log(key + "/actualUpdateRate", navX.getActualUpdateRate());
    log(key + "/firmwareVersion", navX.getFirmwareVersion());
    log(key + "/altitude", navX.getAltitude());
    log(key + "/angle", navX.getAngle());
    log(key + "/angleAdjustment", navX.getAngleAdjustment());
    log(key + "/compassHeading", navX.getCompassHeading());
    log(key + "/displacementX", navX.getDisplacementX());
    log(key + "/displacementY", navX.getDisplacementY());
    log(key + "/displacementZ", navX.getDisplacementZ());
    log(key + "/fusedHeading", navX.getFusedHeading());
    log(key + "/pitch", navX.getPitch());
    log(key + "/pressure", navX.getPressure());
    log(key + "/roll", navX.getRoll());
    log(key + "/yaw", navX.getYaw());
    log(key + "/temperature", navX.getTempC());
    log(key + "/velocityX", navX.getVelocityX());
    log(key + "/velocityY", navX.getVelocityY());
    log(key + "/velocityZ", navX.getVelocityZ());
    log(key + "/accelerationX", navX.getRawAccelX());
    log(key + "/accelerationY", navX.getRawAccelY());
    log(key + "/accelerationZ", navX.getRawAccelZ());
  }

  public static void logStatusSignal(String key, Supplier<StatusSignal<?>> supplier) {
    addUpdate(key, () -> log(key, supplier.get()));
  }

  public static void log(String key, StatusSignal<?> supplier) {
    log(key + "/status", supplier.getStatus().toString());
    logObject(key + "/value", supplier.getValue());
  }

  public static void logCANcoder(String key, Supplier<CANcoder> supplier) {
    addUpdate(key, () -> log(key, supplier.get()));
  }

  public static void log(String key, CANcoder encoder) {
    log(key + "/absolutePosition", encoder.getAbsolutePosition());
    log(key + "/position", encoder.getPosition());
    log(key + "/velocity", encoder.getVelocity());
  }

  public static void logCANStatus(String key, Supplier<CANStatus> supplier) {
    addUpdate(key, () -> log(key, supplier.get()));
  }

  public static void log(String key, CANStatus canStatus) {
    log(key + "/percentBusUtilization", canStatus.percentBusUtilization);
    log(key + "/busOffCount", canStatus.busOffCount);
    log(key + "/txFullCount", canStatus.txFullCount);
    log(key + "/receiveErrorCount", canStatus.receiveErrorCount);
    log(key + "/transmitErrorCount", canStatus.transmitErrorCount);
  }

  public static void logXBoxController(String key, XboxController controller) {
    addUpdate(key, () -> logXBoxControllerOnce(key, controller));
  }

  public static void logXBoxControllerOnce(String key, XboxController controller) {
    log(key + "/port", controller.getPort());

    log(key + "/joystickLeftX", controller.getLeftX());
    log(key + "/joystickLeftY", controller.getLeftY());
    log(key + "/joystickRightX", controller.getRightX());
    log(key + "/joystickRightY", controller.getRightY());

    log(key + "/bumperLeft", controller.getLeftBumperButton());
    log(key + "/bumperRight", controller.getRightBumperButton());
    log(key + "/triggerLeft", controller.getLeftTriggerAxis());
    log(key + "/triggerRight", controller.getRightTriggerAxis());

    log(key + "/buttonA", controller.getAButton());
    log(key + "/buttonB", controller.getBButton());
    log(key + "/buttonX", controller.getXButton());
    log(key + "/buttonY", controller.getYButton());

    log(key + "/buttonStart", controller.getStartButton());
    log(key + "/buttonBack", controller.getBackButton());
    log(key + "/joystickButtonLeft", controller.getLeftStickButton());
    log(key + "/joystickButtonRight", controller.getRightStickButton());

    log(key + "/pov", controller.getPOV() == -1 ? -1 : -controller.getPOV() + 90);
    log(key + "/povRaw", controller.getPOV());
  }

  public static void logRoboRIO(String key) {
    addUpdate(key, () -> logRoboRIOOnce(key));
  }

  public static void logRoboRIOOnce(String key) {
    log(key + "/isBrownedOut", RobotController.isBrownedOut());
    log(key + "/isSysActive", RobotController.isSysActive());
    log(key + "/brownoutVoltage", RobotController.getBrownoutVoltage());
    log(key + "/batteryVoltage", RobotController.getBatteryVoltage());
    log(key + "/batteryVoltage", RobotController.getBatteryVoltage());
    log(key + "/inputCurrent", RobotController.getInputCurrent());
    log(key + "/inputVoltage", RobotController.getInputVoltage());
    log(key + "/3V3Line/current", RobotController.getCurrent3V3());
    log(key + "/5VLine/current", RobotController.getCurrent5V());
    log(key + "/6VLine/current", RobotController.getCurrent6V());
    log(key + "/3V3Line/enabled", RobotController.getEnabled3V3());
    log(key + "/5VLine/enabled", RobotController.getEnabled5V());
    log(key + "/6VLine/enabled", RobotController.getEnabled6V());
    log(key + "/3V3Line/faultCount", RobotController.getFaultCount3V3());
    log(key + "/5VLine/faultCount", RobotController.getFaultCount5V());
    log(key + "/6VLine/faultCount", RobotController.getFaultCount6V());
    log(key + "/3V3Line/voltage", RobotController.getVoltage3V3());
    log(key + "/5VLine/voltage", RobotController.getVoltage5V());
    log(key + "/6VLine/voltage", RobotController.getVoltage6V());
    log(key + "/canStatus", RobotController.getCANStatus());
  }

  public static void logPowerDistribution(String key, Supplier<PowerDistribution> supplier) {
    addUpdate(key, () -> log(key, supplier.get()));
  }

  public static void log(String key, PowerDistribution PDH) {
    log(key + "/faults", PDH.getFaults());
    log(key + "/canId", PDH.getModule());
    for (int i = 0; i <= 23; i++) {
      log(key + "/channels/channel" + i + "Current", PDH.getCurrent(i));
    }
    log(key + "/isSwitchableChannelOn", PDH.getSwitchableChannel());
    log(key + "/temperature", PDH.getTemperature());
    log(key + "/totalCurrent", PDH.getTotalCurrent());
    log(key + "/totalJoules", PDH.getTotalEnergy());
    log(key + "/totalWatts", PDH.getTotalPower());
    log(key + "/voltage", PDH.getVoltage());
  }

  public static void logPowerDistributionFaults(
      String key, Supplier<PowerDistributionFaults> supplier) {
    addUpdate(key, () -> log(key, supplier.get()));
  }

  public static void log(String key, PowerDistributionFaults faults) {
    log(key + "/brownout", faults.Brownout);
    log(key + "/canWarning", faults.CanWarning);

    for (int i = 0; i < 24; i++) {
      log(key + "/channel" + i + "BreakerFault", faults.getBreakerFault(i));
    }

    log(key + "/hardwareFault", faults.HardwareFault);
  }

  private static void logObjectRecurring(String key, Supplier<?> supplier) {
    addUpdate(key, () -> logObject(key, supplier.get()));
  }

  /**
   * Safely logs an unknown type of object to NetworkTables. If the object is a known type, it will
   * be logged using one of the other log methods. If the object is some other type, it will attempt
   * to log each of its fields recursively. No exceptions will be thrown, but access exceptions will
   * be logged in place of the values of inaccessible fields.
   *
   * @param key The key to log the object to
   * @param object The object to log
   */
  public static void logObject(String key, Object object) {
    if (object instanceof Boolean) log(key, (Boolean) object);
    else if (object instanceof String) log(key, (String) object);
    else if (object instanceof Number) log(key, (Number) object);
    else if (object instanceof Measure) log(key, (Measure<?>) object);
    else if (object instanceof boolean[]) log(key, (boolean[]) object);
    else if (object instanceof long[]) log(key, (long[]) object);
    else if (object instanceof double[]) log(key, (double[]) object);
    else if (object instanceof Number[]) log(key, (Number[]) object);
    else if (object instanceof Translation2d) log(key, (Translation2d) object);
    else if (object instanceof Rotation2d) log(key, (Rotation2d) object);
    else if (object instanceof ChassisSpeeds) log(key, (ChassisSpeeds) object);
    else if (object instanceof Pose2d) log(key, (Pose2d) object);
    else if (object instanceof SwerveModuleState) log(key, (SwerveModuleState) object);
    else if (object instanceof SwerveModuleState[]) log(key, (SwerveModuleState[]) object);
    else if (object instanceof SwerveModulePosition) log(key, (SwerveModulePosition) object);
    else if (object instanceof AHRS) log(key, (AHRS) object);
    else if (object instanceof StatusSignal<?>) log(key, (StatusSignal<?>) object);
    else if (object instanceof CANcoder) log(key, (CANcoder) object);
    else if (object instanceof CANStatus) log(key, (CANStatus) object);
    else if (object instanceof PowerDistribution) log(key, (PowerDistribution) object);
    else if (object instanceof PowerDistributionFaults) log(key, (PowerDistributionFaults) object);
    else if (object instanceof Supplier) logObjectRecurring(key, (Supplier<?>) object);
    else if (object instanceof Enum) log(key, object.toString());
    else if (object instanceof Object[]) {
      for (int i = 0; i < ((Object[]) object).length; i++) {
        logObject(key + "/" + i, ((Object[]) object)[i]);
      }
    } else if (object instanceof Collection<?>) {
      int i = 0;
      for (var element : (Collection<?>) object) {
        logObject(key + "/" + i, element);
        i++;
      }
    } else if (object instanceof Map<?, ?>) {
      for (var entry : ((Map<?, ?>) object).entrySet()) {
        logObject(key + "/" + entry.getKey().toString(), entry.getValue());
      }
    } else if (object == null) {
      log(key, "null");
    } else {
      try {
        table.getEntry(key).setValue(object);
      } catch (IllegalArgumentException valueException) {
        try {
          for (var field : object.getClass().getFields()) {
            String subpath = key + "/" + field.getName();

            try {
              logObject(subpath, field.get(object));
            } catch (IllegalAccessException accessException) {
              log(subpath, accessException.getMessage());
            }
          }
        } catch (SecurityException securityException) {
          log(key, securityException.getMessage());
        }
      }
    }
  }

  public static void logEnabledSystems() {
    log("Enabled Systems/Dashboard", ENABLED_SYSTEMS.isDashboardEnabled());
    log("Enabled Systems/Drive", ENABLED_SYSTEMS.isDriveEnabled());
    log("Enabled Systems/Elevator", ENABLED_SYSTEMS.isElevatorEnabled());
    log("Enabled Systems/Funnel", ENABLED_SYSTEMS.isFunnelEnabled());
    log("Enabled Systems/Manipulator", ENABLED_SYSTEMS.isManipulatorEnabled());
  }
}
