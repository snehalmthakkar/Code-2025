package com.team6962.lib.swerve.movement;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.team6962.lib.swerve.SwerveCore;
import com.team6962.lib.swerve.module.SwerveModule;
import com.team6962.lib.utils.MeasureMath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.software.MathUtils;

public class PreciseDrivePositionMovement implements SwerveMovement {
  private SwerveModulePosition[] initialPositions;
  private SwerveModulePosition[] positionDeltas;
  private double[] relativeVelocities;

  public PreciseDrivePositionMovement(
      SwerveModulePosition[] initialPositions,
      SwerveModulePosition[] positionDeltas,
      double[] relativeVelocities) {
    this.initialPositions = initialPositions;
    this.positionDeltas = positionDeltas;
    this.relativeVelocities = relativeVelocities;

    if (initialPositions.length != 4
        || positionDeltas.length != 4
        || relativeVelocities.length != 4) {
      throw new IllegalArgumentException("All input arrays must have a length of 4.");
    }
  }

  private SwerveModulePosition[] getOptimizedPositionTargets(
      SwerveModulePosition[] currentPositions) {
    SwerveModulePosition[] optimizedTargets = new SwerveModulePosition[4];

    for (int i = 0; i < 4; i++) {
      double currentAngleRotations = currentPositions[i].angle.getRotations();
      double unoptimizedTargetAngleRotations = positionDeltas[i].angle.getRotations();
      double initialPositionMeters = initialPositions[i].distanceMeters;
      double positionDeltaMeters = positionDeltas[i].distanceMeters;

      double optimizationAdjustmentRotations =
          MathUtils.floorMod(unoptimizedTargetAngleRotations - currentAngleRotations - 0.5, 1)
              - 0.5;

      if (Math.abs(optimizationAdjustmentRotations) > 0.25) {
        positionDeltaMeters *= -1;
        optimizationAdjustmentRotations -= 0.5 * Math.signum(optimizationAdjustmentRotations);
      }

      SwerveModulePosition optimizedTarget =
          new SwerveModulePosition(
              initialPositionMeters + positionDeltaMeters,
              Rotation2d.fromRotations(currentAngleRotations + optimizationAdjustmentRotations));

      optimizedTargets[i] = optimizedTarget;
    }

    return optimizedTargets;
  }

  private void scaleMaxTo1(double[] values) {
    double max = 0;

    for (int i = 0; i < values.length; i++) {
      max = Math.max(max, values[i]);
    }

    for (int i = 0; i < values.length; i++) {
      values[i] /= max;
    }
  }

  private void abs(double[] values) {
    for (int i = 0; i < values.length; i++) {
      values[i] = Math.abs(values[i]);
    }
  }

  private double[] multiplyAll(double[] values, double scalar) {
    double[] output = new double[values.length];

    for (int i = 0; i < values.length; i++) {
      output[i] = values[i] * scalar;
    }

    return output;
  }

  @Override
  public void execute(SwerveCore drivetrain) {
    SwerveModulePosition[] targets = getOptimizedPositionTargets(drivetrain.getModulePositions());

    boolean needsSteerAdjustment = false;

    for (int i = 0; i < 4; i++) {
      SwerveModule module = drivetrain.getModules()[i];

      Angle targetAngle = targets[i].angle.getMeasure();
      Angle currentAngle = module.getSteerAngle();

      Angle difference = MeasureMath.minAbsDifference(currentAngle, targetAngle);

      if (difference.gt(Degrees.of(5))) {
        needsSteerAdjustment = true;
      }
    }

    if (needsSteerAdjustment) {
      for (int i = 0; i < 4; i++) {
        SwerveModule module = drivetrain.getModules()[i];
        Angle targetAngle = targets[i].angle.getMeasure();

        module.drive(
          SwerveMovement.motionMagicVelocityVoltage.withVelocity(0),
          SwerveMovement.motionMagicExpoVoltage.withPosition(targetAngle));
      }

      return;
    }

    scaleMaxTo1(relativeVelocities);
    abs(relativeVelocities);

    double[] maxVelocities = multiplyAll(relativeVelocities, 80);
    double[] maxAccelerations = multiplyAll(relativeVelocities, 70);

    for (int i = 0; i < 4; i++) {
      SwerveModule module = drivetrain.getModules()[i];

      Angle steerAngle = targets[i].angle.getMeasure();
      Distance drivePosition = Meters.of(targets[i].distanceMeters);
      Angle driveAngle = drivetrain.getConstants().driveMotorMechanismToRotor(drivePosition);

      AngularVelocity maxVelocity = RotationsPerSecond.of(maxVelocities[i]);
      AngularAcceleration maxAcceleration = RotationsPerSecondPerSecond.of(maxAccelerations[i]);

      module.drive(
          SwerveMovement.dynamicMotionMagicVoltage
              .withPosition(driveAngle)
              .withVelocity(maxVelocity)
              .withAcceleration(maxAcceleration),
          SwerveMovement.motionMagicExpoVoltage.withPosition(steerAngle));
    }
  }

  @Override
  public SwerveMovement cleared() {
    return new SpeedsMovement();
  }
}
