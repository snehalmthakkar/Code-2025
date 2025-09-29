// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static edu.wpi.first.units.Units.Milliseconds;

import java.io.InputStream;
import java.util.Map;
import java.util.Properties;

import com.team6962.lib.swerve.SwerveDrive;
import com.team6962.lib.swerve.module.SwerveModule;
import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.telemetry.StatusChecks;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.AutoAlign;
import frc.robot.auto.AutoChooser;
import frc.robot.auto.AutoPickup;
import frc.robot.auto.Autonomous;
import frc.robot.auto.GroundAuto;
import frc.robot.commands.PieceCombos;
import frc.robot.commands.SafeSubsystems;
import frc.robot.constants.Constants.CAN;
import frc.robot.constants.Constants.SWERVE;
import frc.robot.field.StationPositioning.CoralStation;
import frc.robot.subsystems.Controls;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.util.CachedRobotState;
import frc.robot.util.RobotEvent;
import frc.robot.vision.Algae;
import frc.robot.vision.field.TrackingField;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private static RobotContainer instance;
  public static RobotEvent disabledPeriodic = new RobotEvent();

  /**
   * Get the RobotContainer instance (for testing or competition only!)
   *
   * @return
   */
  public static RobotContainer getInstance() {
    return instance;
  }

  public final SwerveDrive swerveDrive;
  public final Manipulator manipulator;
  public final Elevator elevator;
  public final AutoAlign autoAlign;
  public final Autonomous autov3;
  public final GroundAuto groundAuto;
  public final Algae algaeDetector;
  public final PieceCombos pieceCombos;
  public final SafeSubsystems safeties;
  public final Intake intake;
  public final TrackingField trackingField;
  public final AutoPickup autoPickup;
  public final Controls controls;
  public final AutoChooser autoChooser;

  private static PowerDistribution PDH = new PowerDistribution(CAN.PDH, ModuleType.kRev);

  SwerveModule module;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    instance = this;

    var log = DataLogManager.getLog();
    DriverStation.startDataLog(log, true);
    logGitProperties(log);

    CachedRobotState.init();

    LiveWindow.disableAllTelemetry();

    DriverStation.silenceJoystickConnectionWarning(true);

    StatusChecks.Category statusChecks = StatusChecks.under("General");

    statusChecks.add("FMS Attached", () -> DriverStation.isFMSAttached());
    statusChecks.add("DS Attached", () -> DriverStation.isDSAttached());
    statusChecks.add("Joystick 0", () -> DriverStation.isJoystickConnected(0));
    statusChecks.add("Joystick 1", () -> DriverStation.isJoystickConnected(1));
    statusChecks.add("Battery Voltage", () -> RobotController.getBatteryVoltage() > 12.0);
    statusChecks.add("Loop Time", () -> Robot.getComputeTime() <= 0.02);
    statusChecks.add("3V3 Enabled", () -> RobotController.getEnabled3V3());
    statusChecks.add("5V Enabled", () -> RobotController.getEnabled5V());
    statusChecks.add("6V Enabled", () -> RobotController.getEnabled6V());
    statusChecks.add("Sys Time Valid", () -> RobotController.isSystemTimeValid());

    Logger.logEnabledSystems();

    swerveDrive = new SwerveDrive(SWERVE.CONFIG);
    new LEDs();
    controls = new Controls(swerveDrive);

    manipulator = new Manipulator();
    elevator = Elevator.create();
    safeties = new SafeSubsystems(elevator, manipulator);
    pieceCombos = new PieceCombos(elevator, manipulator, safeties);
    autoAlign = new AutoAlign(swerveDrive);
    autov3 = new Autonomous(swerveDrive, manipulator, elevator, pieceCombos);
    algaeDetector = new Algae();
    intake = new Intake(manipulator.grabber);

    trackingField = TrackingField.createInstance();
    trackingField.startLogging();
    autoPickup = new AutoPickup(swerveDrive, controls.getSwerveController(), trackingField);
    groundAuto = new GroundAuto(this);

    // // Configure the trigger bindings
    controls.configureBindings(
        swerveDrive, elevator, manipulator, autoAlign, autov3, pieceCombos, autoPickup, intake);

    NetworkTableEntry refreshButtonEntry =
        NetworkTableInstance.getDefault().getTable("StatusChecks").getEntry("refreshButton");

    statusChecks.timestampAdd("timerChecker", () -> Timer.getFPGATimestamp());

    refreshButtonEntry.setBoolean(false);

    Logger.start(Milliseconds.of(20));

    autoChooser = new AutoChooser(Map.of(
      "Nothing", () -> Commands.none(),
      "Right Side", () -> groundAuto.sideAutonomous(CoralStation.RIGHT),
      "Left Side", () -> groundAuto.sideAutonomous(CoralStation.LEFT),
      "Drive Forward", () -> swerveDrive.drive(new ChassisSpeeds(0.5, 0, 0)),
      "Wheel Size Calibration", () -> swerveDrive.calibrateWheelSize()
    ), "Nothing");
  }

  public Command getAutonomousCommand() {
    return autoChooser.getAutonomousCommand();
  }

  public static double getVoltage() {
    return RobotController.getBatteryVoltage();
  }

  public static double getTotalCurrent() {
    return PDH.getTotalCurrent();
  }

  public static PowerDistribution getPDH() {
    return PDH;
  }

  public void latePeriodic() {
    swerveDrive.latePeriodic();
  }

  public void disabledPeriodic() {
    disabledPeriodic.run();
  }

  public void disabledInit() {}

  public void testInit() {}

  private final void logGitProperties(DataLog log) {
    // Load git properties from classpath
    Properties gitProps = new Properties();
    try (InputStream is = getClass().getClassLoader().getResourceAsStream("git.properties")) {
      if (is != null) {
        gitProps.load(is);

        // Log all git properties
        gitProps.forEach(
            (key, value) -> {
              String propertyPath = "/Metadata/" + key.toString();
              StringLogEntry entry = new StringLogEntry(log, propertyPath);
              entry.append(value.toString());

              // Also print to console for debugging
              System.out.println(key + ": " + value);
            });
      } else {
        System.err.println("git.properties not found in classpath");
      }
    } catch (Exception e) {
      System.err.println("Failed to load git properties: " + e.getMessage());
    }
  }
}
