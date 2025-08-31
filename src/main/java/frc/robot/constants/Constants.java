// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.config.PIDConstants;
import com.team6962.lib.swerve.SwerveConfig;
import com.team6962.lib.utils.MeasureMath;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.util.CachedRobotState;
import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;
import java.util.function.Supplier;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class TEAM_COLOR {
    public static final Supplier<Boolean> IS_BLUE_TEAM =
        () -> CachedRobotState.isBlue().orElse(true);
  }

  // ENABLED SYSTEMS
  public static final class ENABLED_SYSTEMS {
    private static final boolean DRIVE = true;
    private static final boolean DASHBOARD = true;
    private static final boolean HANG = false;
    private static final boolean MANIPULATOR = true;
    private static final boolean ELEVATOR = true;
    private static final boolean FUNNEL = true;

    public static boolean isDriveEnabled() {
      return DRIVE;
    }

    public static boolean isDashboardEnabled() {
      return DASHBOARD;
    }

    public static boolean isHangEnabled() {
      return HANG;
    }

    public static boolean isManipulatorEnabled() {
      return MANIPULATOR;
    }

    public static boolean isElevatorEnabled() {
      return ELEVATOR;
    }

    public static boolean isFunnelEnabled() {
      return FUNNEL;
    }
  }

  public static final class LOGGING {
    public static final int LOGGING_PERIOD_MS = 20;
  }

  // DEVICES
  public static final class DEVICES {
    public static final int DRIVE_XBOX_CONTROLLER = 0;
    public static final int OPERATOR_XBOX_CONTROLLER = 1;
  }

  // DASHBOARD (ShuffleBoard)
  public static final class DASHBOARD {
    public static final String TAB_NAME = "SwerveDrive";
  }

  public static final class ALGAE {
    public static final Distance ALGAE_DIAMETER = Inches.of(16.25); // meters
    public static final boolean SIMULATE_STATIC_ALGAE = false;
  }

  // LIMELIGHT
  // Exposure: 750
  // Sensor Gain: 10
  public static final class LIMELIGHT {
    // x is front-to-back
    // y is left-to-right
    // z it top-to-bottom
    public static final Map<String, Pose3d> APRILTAG_CAMERA_POSES =
        Map.of(
            "limelight-ftag",
                new Pose3d(
                    Units.inchesToMeters(14.5),
                    Units.inchesToMeters(0.0),
                    Units.inchesToMeters(7.5),
                    new Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(19.0), 0.0)),
            "limelight-btag",
                new Pose3d(
                    Units.inchesToMeters(0.0),
                    Units.inchesToMeters(0.0),
                    Units.inchesToMeters(36.0),
                    new Rotation3d(0.0, 0.0, Units.degreesToRadians(180.0))),
            "limelight-rtag",
                new Pose3d(
                    Units.inchesToMeters(10.451441),
                    Units.inchesToMeters(-11.119072),
                    Units.inchesToMeters(8.821067),
                    new Rotation3d(0, 10, 47.758140)),
            "limelight-ltag",
                new Pose3d(
                    Units.inchesToMeters(10.451441),
                    Units.inchesToMeters(11.119072),
                    Units.inchesToMeters(8.621067),
                    new Rotation3d(180, 10, 47.758140)));

    public static final String ALGAE_CAMERA_NAME = "limelight-algae";
    public static final int[] BLACKLISTED_APRILTAGS = {};

    public static final double SPHERE_TOLERANCE = 0.5;

    public static final Rotation2d ALGAE_CAMERA_PITCH =
        Rotation2d.fromDegrees(-24); // CHANGE (DEGREES)
    // x is forward, y is left, z is up
    public static final Translation3d ALGAE_CAMERA_POSITION =
        new Translation3d(
            Units.inchesToMeters(3.0), Units.inchesToMeters(0.0), Units.inchesToMeters(32.5));

    public static final Rotation2d FOV_HEIGHT = Rotation2d.fromDegrees(48.9); // Degrees
    public static final Rotation2d FOV_WIDTH = Rotation2d.fromDegrees(62.5); // Degrees
    public static final double ALGAE_CAMERA_HEIGHT_PIXELS = 960;

    public static final Distance MAX_DETECTION_RANGE =
        Meters.of(19.30); // Max distance an algae can be while being on the field
  }

  // Competition: 5476
  public static final class SWERVE {
    private static final Angle ALL_OFFSET = Degrees.of(135);

    public static final SwerveConfig.Module[] MODULE_CONFIGS = {
      // Back Left: -0.003
      // Back Right: -0.018
      // Front Left: 0.028
      // Front Right: 0.018(needs recalibration)

      new SwerveConfig.Module(
          10,
          20,
          30,
          Radians.of(0.192)
              .minus(Radians.of(0.00959265359))
              .minus(Radians.of(-0.009370614359))
              .minus(Radians.of(0.002629385641))
              .minus(Radians.of(0.018))
              .minus(Radians.of(-0.037))), // Front Right
      new SwerveConfig.Module(
          11,
          21,
          31,
          Radians.of(-1.911)
              .minus(Radians.of(0.00959265359))
              .minus(Radians.of(-0.04577796077))
              .minus(Radians.of(0.028))
              .minus(Radians.of(-0.01259265359))), // Front Left
      new SwerveConfig.Module(
          12,
          22,
          32,
          Radians.of(1.555)
              .minus(Radians.of(0.01577796077))
              .minus(Radians.of(0.002777960769))
              .minus(Radians.of(-0.003370614359))
              .minus(Radians.of(-0.003))
              .minus(Radians.of(-0.012))), // Back Left
      new SwerveConfig.Module(
          13,
          23,
          33,
          Radians.of(0.01540734641)
              .minus(Radians.of(0.08577796077))
              .plus(Radians.of(0.04290734641))), // Back Right
      new SwerveConfig.Module( // Front Right (Test)
          14, 24, 34, Radians.of(-2.439 - Math.PI / 4).plus(ALL_OFFSET).minus(Degrees.of(15.46))),
      new SwerveConfig.Module( // Front Left (Test)
          15,
          25,
          35,
          Radians.of(-0.440 + Math.PI / 2 + Math.PI / 4)
              .plus(ALL_OFFSET)
              .minus(Degrees.of(12.83))
              .plus(Degrees.of(.35))),
      new SwerveConfig.Module( // Back Right (Test)
          16,
          26,
          36,
          Radians.of(-1.842 - Math.PI / 2 - 3.0 / 4.0 * Math.PI)
              .plus(ALL_OFFSET)
              .minus(Degrees.of(15.82))),
      new SwerveConfig.Module( // Back Left (Test)
          17,
          27,
          37,
          Radians.of(-1.049 - Math.PI + 3.0 / 4.0 * Math.PI)
              .plus(ALL_OFFSET)
              .minus(Degrees.of(14.41))
              .minus(Degrees.of(.88))),
      new SwerveConfig.Module(18, 28, 38, Degrees.of(0)),
    };

    public static final SwerveConfig CONFIG = SwerveConstants.get();
  }

  public static final class SWERVE_DRIVE {
    public static final double TELEOPERATED_FINE_TUNE_DRIVE_POWER =
        0.1; // Percent driving power when using d-pad
    public static final double TELEOPERATED_DRIVE_POWER = 0.5; // Percent driving power
    public static final double TELEOPERATED_BOOST_POWER =
        1.0; // Percent power when using the triggers
    public static final double TELEOPERATED_ROTATE_POWER = 0.1; // Percent rotating power
    public static final double TELEOPERATED_ROTATE_BOOST_POWER = 0.2; // Percent rotating power
    public static final double TELEOPERATED_SUPER_ROTATE_POWER = 0.7; // Percent rotating power
    public static final double TELEOPERATED_ULTRA_FINE_TUNE_DRIVE_POWER = 0.5;
  }

  public static final class CANBUS {
    public static final String DRIVETRAIN_CANBUS = "drivetrain";
  }

  public static final class CAN {
    // In order of: front left, front right, back left, back right, where the battery is in the back
    public static final int PDH = 1;
    public static final int HANG = 34;
    public static final int ELEVATOR_LEFT = 2;
    public static final int ELEVATOR_RIGHT = 3;
    public static final int MANIPULATOR_PIVOT = 4;
    public static final int MANIPULATOR_GRABBER = 5;
    public static final int MANIPULATOR_FUNNEL = 6;
  }

  public static final class DIO {
    public static final int HANG_ENCODER = 10;
    public static final int CORAL_CLEAR_BEAM_BREAK = 3;
    public static final int CORAL_DETECT_BEAM_BREAK = 0;
    public static final int ELEVATOR_FLOOR_LIMIT = 1;
    public static final int ELEVATOR_CEIL_LIMIT = 2;
    public static final int MANIPULATOR_ENCODER = 4;
  }

  public static final class PWM {
    public static final int LEDS = 9;
  }

  public static final class NEO {
    public static final DCMotor STATS =
        new DCMotor(12.0, 3.0, 160.0, 2.065, Units.rotationsPerMinuteToRadiansPerSecond(5820), 1);

    public static final double SAFE_TEMPERATURE = 60;
    public static final int SAFE_STALL_CURRENT = 40;
    public static final int SAFE_FREE_CURRENT = 60;
    public static final double SAFE_RAMP_RATE = 0.1;

    public static double maxTorqueCurrentLimited(int currentLimit) {
      return STATS.stallTorqueNewtonMeters / STATS.stallCurrentAmps * currentLimit;
    }
  }

  public static final class NEO550 {
    public static final DCMotor STATS = DCMotor.getNeo550(1);
    public static final double SAFE_TEMPERATURE = 60;
    public static final int SAFE_STALL_CURRENT = 10;
    public static final double SAFE_RAMP_RATE = 0.1;

    //
    public static double maxTorqueCurrentLimited(int currentLimit) {
      return STATS.stallTorqueNewtonMeters / STATS.stallCurrentAmps * currentLimit;
    }
  }

  public static final class KRAKEN {
    public static final DCMotor STATS =
        new DCMotor(12.0, 7.09, 366.0, 2.0, Units.rotationsPerMinuteToRadiansPerSecond(6000), 1);
    public static final int SAFE_STALL_CURRENT = 120;
    public static final int SAFE_FREE_CURRENT = 70;
    public static final double SAFE_RAMP_RATE = 0.1;

    public static double maxTorqueCurrentLimited(int currentLimit) {
      return STATS.stallTorqueNewtonMeters / STATS.stallCurrentAmps * currentLimit;
    }
  }

  public static final class HANG { // Adjust these as needed
    public static final Current MAX_CURRENT = Amps.of(80);
    public static final PIDConstants DEPLOY_PROFILE = new PIDConstants(1.0, 0.0, 0.0);

    public static final Angle ENCODER_OFFSET = Radians.of(0.0);

    public static final Angle HANG_ANGLE = Degrees.of(180.0);
    public static final Angle SWAP_ANGLE = Degrees.of(80.0);
    public static final Angle DEPLOY_ANGLE = Degrees.of(0.0);
    public static final Angle STOW_ANGLE = Degrees.of(110.0);
  }

  public static final boolean SAFETIES_ENABLED = true;

  public static final class ELEVATOR {
    public static final double GEARING = 6.72;
    public static final Distance CYCLE_HEIGHT = Inches.of(2.16 * Math.PI); // CALCULATE
    public static final Distance TOLERANCE = Inches.of(0.5);
    public static final Distance Bhobe_HEIGHT = Inches.of(1);

    public static final class PROFILE {
      public static final double kP = 4.5;
      public static final double kS = 0.9;
    }

    // HEIGHT IS MEASURED FROM THE GROUND TO THE TOP OF THE ELEVATOR
    public static final Distance BASE_HEIGHT = Inches.of(41.50);
    public static final Distance MAX_HEIGHT = Inches.of(70.65);
    public static final Distance MIN_HEIGHT = BASE_HEIGHT;
    public static final Distance STOW_HEIGHT = BASE_HEIGHT;
    public static final Distance MAX_UNLIMITED_HEIGHT = Inches.of(41.0); // AVERAGE

    public static final class CORAL {
      public static final Distance L1_HEIGHT = Inches.of(46.0); // change
      public static final Distance L2_HEIGHT = Inches.of(49.2);
      public static final Distance L3_HEIGHT = Inches.of(56.1);
      public static final Distance L4_HEIGHT = MAX_HEIGHT;
      public static final Distance INTAKE_HEIGHT = MIN_HEIGHT;
    }

    public static final class ALGAE {
      public static final Distance L2_HEIGHT = Inches.of(49.35);
      public static final Distance L3_HEIGHT = Inches.of(56.75);
      public static final Distance BARGE_HEIGHT = MAX_HEIGHT;
      public static final Distance GROUND_HEIGHT = MIN_HEIGHT;
      public static final Distance PROCESSOR_HEIGHT = Inches.of(41.50);
    }

    public static final class AUTO {
      public static final Distance READY_HEIGHT = Inches.of(53);
    }
  }

  public static final class MANIPULATOR_PIVOT {
    public static final double GEARING = (36.0 / 16.0) * (4.0) * (4.0);
    public static final double ROTATION_DELAY = 0.3; // seconds
    public static final Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(0.25);
    public static final Angle ABSOLUTE_POSITION_OFFSET = Rotations.of(0.652);
    public static final Angle CENTER_OF_MASS_OFFSET = Degrees.of(85); // CALCULATED FROM CAD

    public static final class PROFILE {
      public static final double kP = 4.25; // 3.75;
      public static final double kI = 0.0;
      public static final double kD = 0.0; // 10.0; // 0.1;
      public static final double kS = 0.2;
      public static final double MAX_ACCELERATION = 30.0; // rad/s^2
    }

    public static final Angle MAX_ANGLE = Degrees.of(40.0); // RESET TO 40.0
    public static final Angle MIN_ANGLE = Degrees.of(-90.0);

    public static final Angle STOW_ANGLE = Degrees.of(-5.0);
    public static final Angle SAFE_ANGLE = Degrees.of(-5.0);

    // TODO: update for v2
    public static final Angle SAFE_MIN_ANGLE = MIN_ANGLE;
    public static final Angle SAFE_MAX_ANGLE = Degrees.of(0.0);

    public static final Angle PID_MIN_ANGLE = Degrees.of(-90.0);
    public static final Angle PID_MID_ANGLE = Degrees.of(-45.0);
    public static final Angle PID_MAX_ANGLE = Degrees.of(0.0);
    public static final Angle TOLERANCE = Degrees.of(2.0);
    public static final Angle SAFE_TOLERANCE = Degrees.of(4.0);
    public static final boolean INVERTED = true;

    public static final class CORAL {
      public static final Angle L1_ANGLE = Degrees.of(-10.0);
      public static final Angle L23_ANGLE = Degrees.of(-22.35);
      public static final Angle L4_ANGLE = Degrees.of(-48.5);
      public static final Angle INTAKE_ANGLE = Degrees.of(-22.0);
    }

    public static final class ALGAE {
      public static final class BARGE {
        public static final Angle AIM_ANGLE = MAX_ANGLE;
        public static final Angle RELEASE_ANGLE = Degrees.of(0.0);
        public static final Angle END_ANGLE = Degrees.of(-30.0);
      }

      public static final Angle REEF_ANGLE = Degrees.of(-51.2);
      public static final Angle GROUND_ANGLE = Degrees.of(-75.0);
      public static final Angle PROCESSOR_ANGLE = Degrees.of(-10.0);
      public static final Angle HOLD_ANGLE = Degrees.of(-11.5);
    }

    public static final NavigableMap<Distance, Angle> MIN_ANGLES;

    static {
      MIN_ANGLES = new TreeMap<>();
      MIN_ANGLES.put(Inches.of(Double.POSITIVE_INFINITY), MIN_ANGLE);
    }

    public static final NavigableMap<Distance, Angle> MAX_ANGLES;

    static {
      MAX_ANGLES = new TreeMap<>();
      MAX_ANGLES.put(ELEVATOR.MAX_HEIGHT.minus(Inches.of(0.5)), SAFE_MAX_ANGLE);
      MAX_ANGLES.put(Inches.of(Double.POSITIVE_INFINITY), MAX_ANGLE);
    }
  }

  public static final class MANIPULATOR {
    public static final Current ALGAE_DETECT_CURRENT = Amps.of(15);
    public static final Time ALGAE_GRIP_CHECK_TIME = Seconds.of(1.0);
    public static final Time ALGAE_GRIP_IDLE_TIME = Seconds.of(1.0);

    public static final boolean ALGAE_GRIP_CHECK_ENABLED = true;
    public static final double ALGAE_GRIP_CHECK_SPEED = 0.7;

    public static final double ALGAE_OUT_SPEED = -1.0;
    public static final double ALGAE_IN_SPEED = 0.7;
    public static final double ALGAE_HOLD_SPEED = 0.4;

    public static final double CORAL_OUT_SPEED = 0.7;
    public static final double CORAL_IN_SPEED = 0.3;
    public static final double CORAL_SLOW_IN_SPEED = 0.1;
    public static final double CORAL_ADJUST_SPEED = -0.1;
    public static final double CORAL_REPOSITION_SPEED = 0.05;
    public static final double CORAL_HOLD_SPEED = -0.0;
    public static final Time CORAL_ADJUST_TIME = Seconds.of(0.1);

    public static final double BASE_SPEED = 0.5;

    public static final double FUNNEL_IN_SPEED = 0.5;
  }

  public static final class HANG_PIVOT {
    public static final double GEARING = (9.0 / 1.0) * (9.0 / 1.0) * (3.0 / 1.0) * (26.0 / 12.0);
    public static final Angle ENCODER_OFFSET = Rotations.of(0.085);

    public static final Angle MAX_ANGLE = Degrees.of(140.0);
    public static final Angle MIN_ANGLE = Degrees.of(-40.0);
    public static final Angle STOW_ANGLE = Degrees.of(0.0);
    public static final Angle HANG_ANGLE = Degrees.of(-40.0);

    public static final class PROFILE {
      public static final double kP = 0.5;
      public static final double kS = 0.5; // volts per rad/s
      public static final double kD = 0.0;
      public static final double kI = 0.0;
    }
  }

  // LED
  public static final class LED {
    public static final Distance Spacing = Meters.of(1 / 60.0);
    public static final int port = 9;
    public static final int SIDE_STRIP_HEIGHT = 56;
  }

  public static final class FUNNEL {
    public static final boolean MOTOR_INVERTED = false;
    public static final double INTAKE_SPEED = 1.0;
    public static final Time INTAKE_TIME = Seconds.of(0.5);
  }

  public static final class VOLTAGE_LADDER {
    public static final double SWERVE_DRIVE = 7.0;
    public static final double ELEVATOR = 7.5;
    public static final double MANIPULATOR = 8.0;
    public static final double HANG = 9.0;
  }

  public static final class TIMING {
    public static final Time MATCH_LENGTH = MeasureMath.time(2, 30);
    public static final Time AUTO_LENGTH = MeasureMath.time(0, 15);
    public static final Time ENDGAME_LENGTH = MeasureMath.time(0, 20);
    public static final Time ENDGAME_START = MATCH_LENGTH.minus(ENDGAME_LENGTH);
  }

  public static final class AUTO {
    public static final Time WORK_TIME = Milliseconds.of(5);
    public static final Time SLEEP_TIME = Milliseconds.of(15);
  }
}
