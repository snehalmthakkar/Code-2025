package com.team6962.lib.swerve;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.team6962.lib.swerve.auto.AutoBuilderWrapper;
import com.team6962.lib.swerve.auto.PathPrecomputing;
import com.team6962.lib.swerve.controller.HolonomicPositionController;
import com.team6962.lib.swerve.controller.PIDConstraints;
import com.team6962.lib.swerve.module.SwerveModule;
import com.team6962.lib.swerve.movement.ConstantVoltageMovement;
import com.team6962.lib.swerve.movement.PreciseDrivePositionMovement;
import com.team6962.lib.swerve.prepath.CustomPathfindingCommand;
import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.utils.CommandUtils;
import com.team6962.lib.utils.KinematicsUtils;
import com.team6962.lib.utils.MeasureMath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The main class for the swerve drive system. This class extends {@link SwerveCore} to provide the
 * {@link Command}-based API for controlling the drivetrain.
 */
public class SwerveDrive extends SwerveCore {
  /** Subsystem for translation commands to require. */
  private SubsystemBase translationSubsystem = new SubsystemBase() {};

  /** Subsystem for rotation commands to require. */
  private SubsystemBase rotationSubsysem = new SubsystemBase() {};

  private AutoBuilderWrapper autoBuilder = new AutoBuilderWrapper();

  private PathPrecomputing pathPrecomputing;

  /**
   * Create a new SwerveDrive object with the given configuration.
   *
   * @param constants The SwerveConfig object that contains the configuration for the swerve drive.
   */
  public SwerveDrive(SwerveConfig constants) {
    super(constants);

    setName("Swerve Drive");

    pathPrecomputing =
        new PathPrecomputing(getConstants().pathConstraints(), getConstants().pathRobotConfig());

    autoBuilder.configure(
        this::getEstimatedPose,
        this::resetPoseEstimate,
        () -> fieldToRobot(getEstimatedSpeeds()),
        getConstants().driveGains().pathController(),
        getConstants().pathRobotConfig(),
        () -> false);
  }

  @Override
  public void periodic() {
    Field2d field = Logger.getField();

    field.setRobotPose(getEstimatedPose());

    FieldObject2d modules = field.getObject("Swerve Modules");

    modules.setPoses(getModulePoses());
  }

  public SubsystemBase useTranslation() {
    return translationSubsystem;
  }

  public SubsystemBase useRotation() {
    return rotationSubsysem;
  }

  public SubsystemBase[] useMotion() {
    return new SubsystemBase[] {translationSubsystem, rotationSubsysem};
  }

  public LinearVelocity getLinearDriveVelocity(double powerFraction) {
    return getConstants().maxDriveSpeed().times(powerFraction);
  }

  public AngularVelocity getAngularDriveVelocity(double powerFraction) {
    return getConstants().maxRotationSpeed().times(powerFraction);
  }

  public Command driveModules(Supplier<SwerveModuleState[]> states) {
    return Commands.run(() -> moveRobotRelative(states.get()), useMotion());
  }

  public Command drive(SwerveModuleState[] states) {
    return driveModules(() -> states);
  }

  public Command driveSpeeds(Supplier<ChassisSpeeds> speeds) {
    return Commands.run(() -> moveFieldRelative(speeds.get()), useMotion());
  }

  public Command drive(ChassisSpeeds speeds) {
    return driveSpeeds(() -> speeds);
  }

  public Command driveTranslation(Supplier<Translation2d> translation) {
    return Commands.run(() -> moveFieldRelative(translation.get()), useTranslation());
  }

  public Command drive(Translation2d translation) {
    return driveTranslation(() -> translation);
  }

  public Command driveRotation(Supplier<Rotation2d> rotation) {
    return Commands.run(() -> moveRobotRelative(rotation.get()), useRotation());
  }

  public Command drive(Rotation2d rotation) {
    return driveRotation(() -> rotation);
  }

  private class HeadingCommand extends Command {
    private PIDController pid;
    private Supplier<Rotation2d> heading;

    public HeadingCommand(Supplier<Rotation2d> heading) {
      this.heading = heading;

      addRequirements(useRotation());
    }

    @Override
    public void initialize() {
      PIDConstants constants = getConstants().driveGains().rotation();
      pid = new PIDController(constants.kP, constants.kI, constants.kD);

      pid.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void execute() {
      Rotation2d targetHeading = heading.get();
      Rotation2d currentHeading = getEstimatedPose().getRotation();

      if (targetHeading == null) return;

      double output = pid.calculate(currentHeading.getRadians(), targetHeading.getRadians());

      output = MathUtil.clamp(output, -1, 1);

      moveRobotRelative(getAngularDriveVelocity(output));
    }

    @Override
    public void end(boolean interrupted) {
      pid.close();
    }
  }

  public Command driveHeading(Supplier<Rotation2d> heading) {
    return new HeadingCommand(heading);
  }

  public Command driveHeading(Rotation2d heading) {
    return driveHeading(() -> heading);
  }

  public Command facePoint(Supplier<Translation2d> point, Supplier<Rotation2d> offset) {
    return driveHeading(
        () -> {
          Translation2d currentPoint = point.get();

          if (currentPoint == null) return null;

          Translation2d currentPoseTranslation = getEstimatedPose().getTranslation();
          Translation2d translationDistance = currentPoint.minus(currentPoseTranslation);
          Rotation2d targetHeading = translationDistance.getAngle().plus(offset.get());

          return targetHeading;
        });
  }

  public Command facePoint(Supplier<Translation2d> point) {
    return facePoint(point, () -> Rotation2d.fromRotations(0));
  }

  public Command facePoint(Translation2d point) {
    return facePoint(() -> point);
  }

  public Command stop() {
    return driveModules(() -> KinematicsUtils.getStoppedStates(getModuleStates()));
  }

  public Command park() {
    return driveModules(() -> KinematicsUtils.getParkedStates());
  }

  public Command pathfindTo(Pose2d target) {
    return Commands.defer(
        () -> {
          autoBuilder.setOutput(this::moveRobotRelative);

          return CommandUtils.withRequirements(
              AutoBuilder.pathfindToPose(target, getConstants().pathConstraints()), useMotion());
        },
        Set.of(useMotion()));
  }

  public Command pathfindTo(Pose2d target, LinearVelocity goalEndVelocity) {
    autoBuilder.setOutput(this::moveRobotRelative);

    return CommandUtils.withRequirements(
        AutoBuilder.pathfindToPose(target, getConstants().pathConstraints(), goalEndVelocity),
        useMotion());
  }

  public Command pathfindTo(Translation2d target) {
    autoBuilder.setOutput(this::moveRobotRelative);

    return CommandUtils.withRequirements(
        AutoBuilder.pathfindToPose(
            new Pose2d(target, Rotation2d.fromRotations(0)), getConstants().pathConstraints()),
        useMotion());
  }

  public Command pathfindTo(Translation2d target, LinearVelocity goalEndVelocity) {
    autoBuilder.setOutput(this::moveRobotRelative);

    return CommandUtils.withRequirements(
        AutoBuilder.pathfindToPose(
            new Pose2d(target, Rotation2d.fromRotations(0)),
            getConstants().pathConstraints(),
            goalEndVelocity),
        useMotion());
  }

  public boolean isWithinToleranceOf(
      Supplier<Pose2d> targetSupplier, Distance toleranceDistance, Angle toleranceAngle) {
    Pose2d target = targetSupplier.get();
    Pose2d current = getEstimatedPose();

    return current.getTranslation().getDistance(target.getTranslation())
            < toleranceDistance.in(Meters)
        && MeasureMath.minAbsDifference(
                current.getRotation().getMeasure(), target.getRotation().getMeasure())
            .lt(toleranceAngle);
  }

  public boolean isWithinToleranceOf(
      Pose2d target, Distance toleranceDistance, Angle toleranceAngle) {
    return isWithinToleranceOf(() -> target, toleranceDistance, toleranceAngle);
  }

  public Command pathfindBetweenWaypoints(
      Pose2d startPose,
      Pose2d endPose,
      Rotation2d startVelocityAngle,
      Rotation2d endVelocityAngle,
      LinearVelocity expectedStartVelocity,
      LinearVelocity maxEndVelocity) {
    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(
            List.of(
                new Pose2d(startPose.getTranslation(), startVelocityAngle),
                new Pose2d(endPose.getTranslation(), endVelocityAngle)));

    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            getConstants().pathConstraints(),
            new IdealStartingState(expectedStartVelocity, startPose.getRotation()),
            new GoalEndState(maxEndVelocity.in(MetersPerSecond), endPose.getRotation()));

    path.getIdealTrajectory(getConstants().pathRobotConfig());

    return CommandUtils.withRequirements(
        new FollowPathCommand(
            path,
            this::getEstimatedPose,
            () -> fieldToRobot(getEstimatedSpeeds()),
            (speeds, ff) -> {
              moveRobotRelative(speeds);
            },
            getConstants().driveGains().pathController(),
            getConstants().pathRobotConfig(),
            () -> false),
        useMotion());
  }

  public Command pathfindToWaypoint(
      Pose2d precomputedStartPose,
      Pose2d endPose,
      Rotation2d precomputedStartVelocityAngle,
      Rotation2d endVelocityAngle,
      LinearVelocity precomputedStartVelocity,
      LinearVelocity maxEndVelocity) {
    Command pregenerated =
        pathfindBetweenWaypoints(
            precomputedStartPose,
            endPose,
            precomputedStartVelocityAngle,
            endVelocityAngle,
            precomputedStartVelocity,
            maxEndVelocity);

    return Commands.sequence(
        Commands.defer(
            () -> {
              Command attachment =
                  pathfindBetweenWaypoints(
                      getEstimatedPose(),
                      precomputedStartPose,
                      precomputedStartPose
                          .getTranslation()
                          .minus(getEstimatedPose().getTranslation())
                          .getAngle(),
                      precomputedStartVelocityAngle,
                      MetersPerSecond.of(
                          KinematicsUtils.getTranslation(getEstimatedSpeeds()).getNorm()),
                      precomputedStartVelocity);

              return attachment;
            },
            Set.of(useMotion())),
        pregenerated);
  }

  public Command pathfindToPrecomputed(Pose2d startPose, Pose2d endPose) {
    PathPrecomputing.Precompute precompute = pathPrecomputing.precompute(startPose, endPose);

    return Commands.defer(
        () -> {
          CustomPathfindingCommand pathfindToPose =
              new CustomPathfindingCommand(
                  endPose,
                  getConstants().pathConstraints(),
                  this::getEstimatedPose,
                  () -> fieldToRobot(getEstimatedSpeeds()),
                  (speeds, ff) -> moveRobotRelative(speeds),
                  getConstants().driveGains().pathController(),
                  getConstants().pathRobotConfig(),
                  useMotion());

          if (precompute.isAvailable() && precompute.isStartPoseNear(getEstimatedPose())) {
            System.out.println("Using precomputed path!");
            pathfindToPose.currentPath = precompute.getPath();
            pathfindToPose.currentTrajectory = precompute.getTrajectory();
            pathfindToPose.timer.restart();

            List<Waypoint> waypoints = precompute.getPath().getWaypoints();

            for (int i = 0; i < waypoints.size(); i++) {
              System.out.println(" (waypoint " + i + ") " + waypoints.get(i).anchor());
            }

            System.out.println(" (start) " + precompute.getStartPose());
            System.out.println(" (end) " + precompute.getEndPose());
            System.out.println(" (current) " + getEstimatedPose());
          } else {
            System.out.println("Using non-precomputed path.");

            System.out.println(" (current) " + getEstimatedPose());
            System.out.println(" (end) " + endPose);
          }

          return pathfindToPose;
        },
        Set.of(useMotion()));
  }

  public Command calibrateWheelSize() {
    return calibrateWheelSize(Seconds.of(10));
  }

  public Command calibrateWheelSize(Time timeout) {
    return Commands.sequence(
            Commands.waitSeconds(1),
            Commands.defer(
                    () ->
                        new Command() {
                          Angle initialGyroAngle;
                          Angle[] initialWheelAngles = new Angle[4];

                          @Override
                          public void initialize() {
                            initialGyroAngle = getContinuousGyroscopeAngle();

                            for (int i = 0; i < 4; i++) {
                              initialWheelAngles[i] = getModules()[i].getDriveWheelAngle();
                            }
                          }

                          @Override
                          public void end(boolean interrupted) {
                            Angle gyroAngleChange =
                                Rotations.of(
                                    getContinuousGyroscopeAngle()
                                        .minus(initialGyroAngle)
                                        .in(Rotations));
                            Distance driveRadius = getConstants().chassis().driveRadius();

                            Logger.log(
                                "Swerve Drive/Wheel Size Calibration/Gyro Angle Change",
                                gyroAngleChange);

                            Angle[] wheelAngleChanges = new Angle[4];

                            for (int i = 0; i < 4; i++) {
                              wheelAngleChanges[i] =
                                  Rotations.of(
                                      getModules()[i]
                                          .getDriveWheelAngle()
                                          .minus(initialWheelAngles[i])
                                          .abs(Rotations));

                              Logger.log(
                                  "Swerve Drive/Wheel Size Calibration/Wheel Angle Change "
                                      + SwerveModule.getModuleName(i),
                                  wheelAngleChanges[i]);
                            }

                            Distance[] wheelDiameters = new Distance[4];

                            for (int i = 0; i < 4; i++) {
                              wheelDiameters[i] =
                                  driveRadius
                                      .times(gyroAngleChange.div(wheelAngleChanges[i]))
                                      .times(2);

                              Logger.log(
                                  "Swerve Drive/Wheel Size Calibration/Diameter "
                                      + SwerveModule.getModuleName(i),
                                  wheelDiameters[i]);
                            }

                            Distance averageDiameter = Meters.of(0);

                            for (Distance diameter : wheelDiameters) {
                              averageDiameter = averageDiameter.plus(diameter);
                            }

                            averageDiameter = averageDiameter.div(4);

                            Logger.log(
                                "Swerve Drive/Wheel Size Calibration/Average Diameter",
                                averageDiameter);
                          }
                        },
                    Set.of())
                .withTimeout(timeout),
            Commands.waitSeconds(1))
        .deadlineFor(
            drive(
                Rotation2d.fromDegrees(
                    getConstants().maxRotationSpeed().div(8).in(DegreesPerSecond))));
  }

  public Command followChoreoPath(String name) {
    PathPlannerPath path;

    try {
      path = PathPlannerPath.fromChoreoTrajectory(name);
    } catch (Exception e) {
      return Commands.none();
    }

    return Commands.defer(
        () -> {
          autoBuilder.setOutput(this::moveRobotRelative);

          return CommandUtils.withRequirements(AutoBuilder.followPath(path), useMotion());
        },
        Set.of(useMotion()));
  }

  public Command constantVoltage(Voltage steerVoltage, Voltage driveVoltage, boolean singleModule) {
    ConstantVoltageMovement movement =
        new ConstantVoltageMovement()
            .withSteerVoltage(steerVoltage)
            .withDriveVoltage(driveVoltage)
            .withSingleModule(singleModule);

    return Commands.run(
        () -> {
          currentMovement = movement;
        },
        useMotion());
  }

  private PreciseDrivePositionMovement createPositionDeltaMovement(Twist2d twist) {
    SwerveModuleState[] states =
        getKinematics().toSwerveModuleStates(new ChassisSpeeds(twist.dx, twist.dy, twist.dtheta));

    SwerveModulePosition[] initial = getModulePositions();
    SwerveModulePosition[] deltas = KinematicsUtils.toModulePositions(states, Seconds.of(1));
    double[] relativeVelocities = KinematicsUtils.getVelocitiesMetersPerSecond(states);

    return new PreciseDrivePositionMovement(initial, deltas, relativeVelocities);
  }

  private SwerveModuleState[] getAlignedStates(Twist2d twist) {
    SwerveModuleState[] states =
        getKinematics().toSwerveModuleStates(new ChassisSpeeds(twist.dx, twist.dy, twist.dtheta));

    return KinematicsUtils.getStoppedStates(states);
  }

  public Command driveTwist(Supplier<Twist2d> twist) {
    return Commands.sequence(
        driveModules(() -> getAlignedStates(twist.get()))
            .until(
                () ->
                    KinematicsUtils.isSimilarAngles(
                        getModuleStates(), getAlignedStates(twist.get()), Degrees.of(3))),
        Commands.run(
            () -> {
              currentMovement = createPositionDeltaMovement(twist.get());
            },
            useMotion()));
  }

  public Command drivePreciselyTo(Pose2d targetPose) {
    return driveTwist(() -> getEstimatedPose().log(targetPose))
      .deadlineFor(Commands.run(() -> {
        Logger.getField().getObject("Target Pose").setPose(targetPose);
      }));
  }

  private class ProfiledDriveCommand extends Command {
    private final HolonomicPositionController controller;
    private HolonomicPositionController.State startState;
    private HolonomicPositionController.State targetState;
    private double startTime;

    public ProfiledDriveCommand(
      HolonomicPositionController.State targetState, // TODO: Convert to supplier or make wrapper methods use Commands.defer
      HolonomicPositionController controller
    ) {
      this.targetState = targetState;
      this.controller = controller;
    }

    @Override
    public void initialize() {
      startState = new HolonomicPositionController.State(getEstimatedPose(), getEstimatedSpeeds());
      startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
      HolonomicPositionController.State currentState =
          new HolonomicPositionController.State(getEstimatedPose(), getEstimatedSpeeds());

      double time = Timer.getFPGATimestamp() - startTime;

      ChassisSpeeds speeds = controller.calculate(time, startState, targetState, currentState);

      moveFieldRelative(speeds);
    }

    @Override
    public boolean isFinished() {
      return controller.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        controller.close();
    }
  }

  public Command driveQuicklyTo(HolonomicPositionController.State targetState) {
    return new ProfiledDriveCommand(targetState, new HolonomicPositionController(
      new TrapezoidProfile.Constraints(
        getConstants().maxDriveSpeed().in(MetersPerSecond) * 0.9,
        getConstants().maxLinearAcceleration().in(MetersPerSecondPerSecond) * 0.9
      ),
      new PIDConstraints(10.0, 0.0, 0.5),
      new TrapezoidProfile.Constraints(
        getConstants().maxRotationSpeed().in(RadiansPerSecond) * 0.9,
        getConstants().maxAngularAcceleration().in(RadiansPerSecondPerSecond) * 0.9
      ),
      new PIDConstraints(3.5, 0.0, 1.5)
    ))
      .deadlineFor(Commands.run(() -> {
        Logger.getField().getObject("Target Pose").setPose(targetState.position);
      }));
  }

  public Command driveQuicklyTo(Pose2d targetPose, ChassisSpeeds targetSpeeds) {
    return driveQuicklyTo(new HolonomicPositionController.State(targetPose, targetSpeeds));
  }

  public Command driveQuicklyTo(Pose2d targetPose) {
    return driveQuicklyTo(targetPose, new ChassisSpeeds());
  }

  public Command driveTo(Pose2d targetPose, ChassisSpeeds targetSpeeds) {
    return driveQuicklyTo(targetPose, targetSpeeds)
      .andThen(drivePreciselyTo(targetPose));
  }

  public Command driveTo(Pose2d targetPose) {
    return driveTo(targetPose, new ChassisSpeeds());
  }
}
