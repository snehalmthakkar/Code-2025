// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.team6962.lib.swerve.SwerveDrive;
import com.team6962.lib.swerve.auto.RobotCoordinates;
import com.team6962.lib.telemetry.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.SWERVE_DRIVE;
import frc.robot.constants.Constants.TEAM_COLOR;
import frc.robot.util.CachedRobotState;
import frc.robot.util.software.MathUtils;
import frc.robot.util.software.MathUtils.InputMath;

public class XBoxSwerve extends Command {
  private XboxController controller;
  private SwerveDrive swerveDrive;

  public double MAX_DRIVE_VELOCITY;
  public double NOMINAL_DRIVE_VELOCITY;
  public double FINE_TUNE_DRIVE_VELOCITY;
  public double ULTRA_FINE_TUNE_DRIVE_VELOCITY;
  public double NOMINAL_ANGULAR_VELOCITY;
  public double MAX_ANGULAR_VELOCITY;
  public double SUPER_ANGULAR_VELOCITY;

  private Translation2d velocity = new Translation2d();
  private double angularVelocity = 0.0;

  private Command translateCommand;
  private Command rotateCommand;

  public XBoxSwerve(SwerveDrive swerveDrive, XboxController xboxController) {
    this.swerveDrive = swerveDrive;
    this.controller = xboxController;
    addRequirements(swerveDrive);

    MAX_DRIVE_VELOCITY =
        swerveDrive
            .getLinearDriveVelocity(SWERVE_DRIVE.TELEOPERATED_BOOST_POWER)
            .in(MetersPerSecond);
    NOMINAL_DRIVE_VELOCITY =
        swerveDrive
            .getLinearDriveVelocity(SWERVE_DRIVE.TELEOPERATED_DRIVE_POWER)
            .in(MetersPerSecond);
    FINE_TUNE_DRIVE_VELOCITY =
        swerveDrive
            .getLinearDriveVelocity(SWERVE_DRIVE.TELEOPERATED_FINE_TUNE_DRIVE_POWER)
            .in(MetersPerSecond);
    ULTRA_FINE_TUNE_DRIVE_VELOCITY =
        swerveDrive
            .getLinearDriveVelocity(SWERVE_DRIVE.TELEOPERATED_ULTRA_FINE_TUNE_DRIVE_POWER)
            .in(MetersPerSecond);
    NOMINAL_ANGULAR_VELOCITY =
        swerveDrive
            .getAngularDriveVelocity(SWERVE_DRIVE.TELEOPERATED_ROTATE_POWER)
            .in(RadiansPerSecond);
    MAX_ANGULAR_VELOCITY =
        swerveDrive
            .getAngularDriveVelocity(SWERVE_DRIVE.TELEOPERATED_ROTATE_BOOST_POWER)
            .in(RadiansPerSecond);
    SUPER_ANGULAR_VELOCITY =
        swerveDrive
            .getAngularDriveVelocity(SWERVE_DRIVE.TELEOPERATED_SUPER_ROTATE_POWER)
            .in(RadiansPerSecond);

    Logger.logNumber("XBoxSwerve/nomVel", () -> NOMINAL_DRIVE_VELOCITY);
    Logger.logNumber("XBoxSwerve/maxVel", () -> MAX_DRIVE_VELOCITY);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!CachedRobotState.isTeleop()) return;

    // Disable drive if the controller disconnects
    if (!controller.isConnected()) {
      if (translateCommand != null) translateCommand.cancel();
      if (rotateCommand != null) rotateCommand.cancel();

      return;
    }

    double leftTrigger = controller.getLeftTriggerAxis();
    double rightTrigger = controller.getRightTriggerAxis();
    Translation2d leftStick = new Translation2d(-controller.getLeftY(), -controller.getLeftX());
    Translation2d rightStick = new Translation2d(controller.getRightX(), -controller.getRightY());

    if (RobotBase.isSimulation()) {
      leftStick = new Translation2d(controller.getRawAxis(0), -controller.getRawAxis(1));
      rightStick = new Translation2d(controller.getRawAxis(2), -controller.getRawAxis(3));

      if (RobotCoordinates.isAllianceInverted().orElse(false)) {
        leftStick = leftStick.unaryMinus();
      }

      leftTrigger = controller.getRawAxis(5);
      rightTrigger = controller.getRawAxis(4);
    }

    // Deadbands
    leftStick = InputMath.addCircularDeadband(leftStick, 0.1);
    rightStick = InputMath.addCircularDeadband(rightStick, 0.1);

    if (leftTrigger > 0.1) {
      angularVelocity +=
          -rightStick.getX()
              * MathUtils.map(leftTrigger, 0, 1, NOMINAL_ANGULAR_VELOCITY, SUPER_ANGULAR_VELOCITY);
    } else {
      angularVelocity +=
          -rightStick.getX()
              * MathUtils.map(rightTrigger, 0, 1, NOMINAL_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
    }

    velocity =
        velocity.plus(
            leftStick.times(
                MathUtils.map(rightTrigger, 0, 1, NOMINAL_DRIVE_VELOCITY, MAX_DRIVE_VELOCITY)));

    if (controller.getPOV() != -1) {
      Rotation2d povDirection = Rotation2d.fromDegrees(controller.getPOV()).unaryMinus();

      Translation2d povVelocity =
          new Translation2d(1.0, 0)
              .rotateBy(povDirection)
              .times(
                  MathUtils.map(
                      rightTrigger,
                      0,
                      1,
                      FINE_TUNE_DRIVE_VELOCITY,
                      ULTRA_FINE_TUNE_DRIVE_VELOCITY));

      povVelocity = swerveDrive.robotToField(povVelocity);

      velocity = velocity.plus(povVelocity);
    }

    // Zero heading when right bumper is pressed
    if (controller.getRightBumperButton()) {
      swerveDrive.resetHeadingEstimate(Rotation2d.fromDegrees(0));
    }

    if (RobotBase.isSimulation()) {
      if (TEAM_COLOR.IS_BLUE_TEAM.get()) {
        velocity = velocity.rotateBy(Rotation2d.fromDegrees(90.0));
      } else {
        velocity = velocity.rotateBy(Rotation2d.fromDegrees(-90.0));
      }
    }

    ChassisSpeeds drivenSpeeds =
        new ChassisSpeeds(velocity.getX(), velocity.getY(), angularVelocity);

    Logger.log("XBoxSwerve/drivenSpeeds", drivenSpeeds);

    boolean movingTranslation = Math.abs(velocity.getNorm()) > 0.05;

    Command currentTranslateCommand = swerveDrive.useTranslation().getCurrentCommand();

    if (currentTranslateCommand == translateCommand
        || currentTranslateCommand == null
        || movingTranslation) {
      if (translateCommand != null) translateCommand.cancel();

      translateCommand = swerveDrive.drive(velocity);
      translateCommand.setName("XBoxSwerveTranslate");
      translateCommand.schedule();
    }

    boolean movingRotation = Math.abs(angularVelocity) > Units.degreesToRadians(3);
    Command currentRotateCommand = swerveDrive.useRotation().getCurrentCommand();

    if (currentRotateCommand == rotateCommand || currentRotateCommand == null || movingRotation) {
      if (rotateCommand != null) rotateCommand.cancel();

      rotateCommand = swerveDrive.drive(Rotation2d.fromRadians(angularVelocity));
      rotateCommand.setName("XBoxSwerveRotate");
      rotateCommand.schedule();
    }

    angularVelocity = 0.0;
    velocity = new Translation2d();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (rotateCommand != null) rotateCommand.cancel();
    if (translateCommand != null) translateCommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
