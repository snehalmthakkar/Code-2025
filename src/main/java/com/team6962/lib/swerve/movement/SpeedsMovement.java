package com.team6962.lib.swerve.movement;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.team6962.lib.swerve.SwerveCore;
import com.team6962.lib.swerve.module.SwerveModule;
import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.utils.KinematicsUtils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Represents a movement that a swerve drive can take over a single control loop cycle.
 *
 * <p>Movements can be created directly with {@link #setStates(SwerveModuleState[])} or {@link
 * #setSpeeds(ChassisSpeeds)}, or by setting the translation and rotation speeds independently with
 * {@link #setTranslation(Translation2d)} and {@link #setRotation(Rotation2d) setRotation()}. The
 * movement can then be converted into an array of {@link SwerveModuleState}s with {@link
 * #getStates()}. To reuse a movement object, call {@link #clear()} to delete it's state.
 *
 * <p>{@code SwerveMovement} was initially implemented as an interface, but was changed to a class
 * for optimization purposes. It's internals should be hidden, in case it needs to be changed back
 * to an interface.
 */
public class SpeedsMovement implements SwerveMovement {
  private SwerveModuleState[] states;
  private ChassisSpeeds speeds;
  private SwerveModuleState[] executedStates = new SwerveModuleState[4];

  /**
   * Sets the swerve module states of the movement, overriding any previous states or speeds.
   *
   * @param states The swerve module states, directions and speeds of individual modules.
   */
  public void setStates(SwerveModuleState[] states) {
    this.states = states;
    speeds = null;
  }

  /**
   * Sets the chassis speeds of the movement, overriding any previous states or speeds.
   *
   * @param speeds The chassis speeds
   */
  public void setSpeeds(ChassisSpeeds speeds) {
    this.speeds = speeds;
    states = null;
  }

  /**
   * Sets the translation speed of the movement, overriding any previous translation or rotation
   * speeds.
   *
   * @param translation The translation speed
   */
  public void setTranslation(Translation2d translation) {
    speeds = KinematicsUtils.setTranslationSpeed(speeds, translation);
  }

  /**
   * Sets the rotation speed of the movement, overriding any previous translation or rotation
   * speeds.
   *
   * @param rotation The rotation speed
   */
  public void setRotation(Rotation2d rotation) {
    speeds = KinematicsUtils.setRotationSpeed(speeds, rotation);
  }

  /**
   * Converts the movement into an array of swerve module states.
   *
   * @return The swerve module states of the movement.
   */
  private SwerveModuleState[] getStates(SwerveCore drivetrain) {
    if (speeds != null) {
      ChassisSpeeds outputSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
      
      Translation2d translation = KinematicsUtils.getTranslation(outputSpeeds);
      
      // TODO: Use data from testing to improve this
      translation = translation.rotateBy(Rotation2d.fromRadians(-outputSpeeds.omegaRadiansPerSecond * 0.142));

      outputSpeeds =
          new ChassisSpeeds(translation.getX(), translation.getY(), outputSpeeds.omegaRadiansPerSecond);

      states = drivetrain.getKinematics().toSwerveModuleStates(outputSpeeds);
    }

    return states;
  }

  @Override
  public void log() {
    for (int i = 0; i < 4; i++) {
      if (executedStates[i] != null) {
        Logger.log(SwerveModule.getModuleName(i) + " Swerve Module/targetState", executedStates[i]);
      }
    }
  }

  @Override
  public void execute(SwerveCore drivetrain) {
    SwerveModule[] modules = drivetrain.getModules();
    LinearVelocity maxLinearVelocity = drivetrain.getMaxDriveSpeed();
    SwerveModuleState[] states = getStates(drivetrain);

    if (states == null) {
      states = KinematicsUtils.getStoppedStates(drivetrain.getModuleStates());
    } else {
      KinematicsUtils.desaturateWheelSpeeds(states, maxLinearVelocity);
    }

    for (int i = 0; i < 4; i++) {
      SwerveModule module = modules[i];
      SwerveModuleState targetState = states[i];

      targetState = SwerveModule.optimizeStateForTalon(targetState, module.getSteerAngle());

      if (Math.abs(targetState.speedMetersPerSecond) < 1e-13) {
        targetState = new SwerveModuleState(0, targetState.angle);
      }

      executedStates[i] = targetState;

      AngularVelocity driveVelocity =
          module
              .getDrivetrainConstants()
              .driveMotorMechanismToRotor(MetersPerSecond.of(targetState.speedMetersPerSecond));

      Angle steerAngle = targetState.angle.getMeasure();

      module.drive(
          SwerveMovement.motionMagicVelocityVoltage.withVelocity(driveVelocity),
          RobotBase.isReal()
              ? SwerveMovement.motionMagicExpoVoltage.withPosition(steerAngle)
              : SwerveMovement.positionVoltage.withPosition(steerAngle));
    }
  }

  /** Clears the movement, deleting any states or speeds that were set. */
  @Override
  public SpeedsMovement cleared() {
    states = null;
    speeds = null;

    return this;
  }
}
