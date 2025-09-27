// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Milliseconds;

import java.io.InputStream;
import java.util.Properties;

import com.team6962.lib.swerve.SwerveDrive;
import com.team6962.lib.swerve.module.SwerveModule;
import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.telemetry.StatusChecks;

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
import frc.robot.auto.AutoAlign;
import frc.robot.auto.Autonomous;
import frc.robot.commands.PieceCombos;
import frc.robot.commands.SafeSubsystems;
import frc.robot.constants.Constants.CAN;
import frc.robot.constants.Constants.SWERVE;
import frc.robot.subsystems.Controls;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.hang.Hang;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.newelevator.NewElevator;
import frc.robot.subsystems.newelevator.SimElevator;
import frc.robot.util.CachedRobotState;
import frc.robot.util.RobotEvent;
import frc.robot.vision.Algae;

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
  public final Hang hang;
  public final AutoAlign autoAlign;
  public final Autonomous autov3;
  public final Algae algaeDetector;
  private final LEDs ledStrip;
  public final PieceCombos pieceCombos;
  public final SafeSubsystems safeties;
  private final Command autonomousCommand;
  public final NewElevator newElevator = new SimElevator();
  private final Intake intake;

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
    ledStrip = new LEDs();

    manipulator = new Manipulator();
    elevator = Elevator.create();
    safeties = new SafeSubsystems(elevator, manipulator);
    pieceCombos = new PieceCombos(elevator, manipulator, safeties);
    autoAlign = new AutoAlign(swerveDrive);
    autov3 = new Autonomous(swerveDrive, manipulator, elevator, pieceCombos);
    algaeDetector = new Algae();
    hang = Hang.create();
    intake = new Intake(manipulator.grabber);

    // // Configure the trigger bindings
    Controls.configureBindings(
        swerveDrive, elevator, manipulator, hang, autoAlign, autov3, pieceCombos, intake);

    NetworkTableEntry refreshButtonEntry =
        NetworkTableInstance.getDefault().getTable("StatusChecks").getEntry("refreshButton");

    statusChecks.timestampAdd("timerChecker", () -> Timer.getFPGATimestamp());

    refreshButtonEntry.setBoolean(false);

    Logger.start(Milliseconds.of(20));

    autonomousCommand = createAutonomousCommand();
  }

  private Command createAutonomousCommand() {
    return newElevator.moveToPosition(Inches.of(60));
    // AUTO ROUTINES - Uncomment the one you want to run

    // 1. Start in the middle of the field, then score 3 coral on the right side.
    // return autov3.createSideAutonomous(Side.RIGHT, true);

    // 2. Start in the middle of the field, then score 3 coral on the left side.
    // return autov3.createSideAutonomous(Side.LEFT, true);

    // 3. Start on the right side of the field, then score 3 coral on the right side.
    // return autov3.createSideAutonomous(Side.RIGHT, false);

    // 4. Start on the left side of the field, then score 3 coral on the left side.
    // return autov3.createSideAutonomous(Side.LEFT, false);

    // 5. Start in the middle field, score preloaded coral on the back face then
    //    throw the algae on the back face into the barge.
    // return autov3.createMiddleAutonomous();

    // 6. Drive foreward until astop
    // return swerveDrive.drive(new ChassisSpeeds(0.5, 0, 0));

    // 7. Do nothing
    // return Commands.none();

    // 8. Old autonomous
    // return autoGen.getCommand();

    ////////////////////////////////////////////////////////////////////////////
    // OTHER COMMANDS - Do not use in matches

    // 9. Calibrate wheel size for odometry
    // return swerveDrive.calibrateWheelSize();
  }

  public Command getAutonomousCommand() {
    return autonomousCommand;
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
