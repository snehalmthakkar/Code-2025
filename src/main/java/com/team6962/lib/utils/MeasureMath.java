package com.team6962.lib.utils;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.util.software.MathUtils;

public final class MeasureMath {
  private MeasureMath() {}

  /**
   * Equivalent to Math.min of two measures.
   *
   * @param <U> The unit type (inferred from the first measure)
   * @param <M> The measure type (inferred from the first measure)
   * @param a The first measure
   * @param b The second measure
   * @return The smaller of the two measures, of type M
   */
  @SuppressWarnings("unchecked")
  public static <U extends Unit, M extends Measure<U>> M min(M a, Measure<U> b) {
    return (M) a.unit().of(Math.min(a.in(a.unit()), b.in(a.unit())));
  }

  /**
   * Equivalent to Math.max of two measures.
   *
   * @param <U> The unit type (inferred from the first measure)
   * @param <M> The measure type (inferred from the first measure)
   * @param a The first measure
   * @param b The second measure
   * @return The larger of the two measures, of type M
   */
  @SuppressWarnings("unchecked")
  public static <U extends Unit, M extends Measure<U>> M max(M a, Measure<U> b) {
    return (M) a.unit().of(Math.max(a.in(a.unit()), b.in(a.unit())));
  }

  /**
   * Clamps a measure between two other measures, equivalent to MathUtil.clamp(value, min, max) or
   * Math.min(Math.max(value, min), max).
   *
   * @param <U> The unit type (inferred from value)
   * @param <M> The measure type (inferred from value)
   * @param value The value to clamp
   * @param min The minimum value
   * @param max The maximum value
   * @return The clamped value, of type M
   */
  @SuppressWarnings("unchecked")
  public static <U extends Unit, M extends Measure<U>> M clamp(
      M value, Measure<U> min, Measure<U> max) {
    return (M)
        value
            .unit()
            .of(
                Math.min(
                    Math.max(value.in(value.unit()), min.in(value.unit())), max.in(value.unit())));
  }

  public static Distance arcLength(Distance radius, Angle angle) {
    return Meters.of(radius.in(Meters) * angle.in(Radians));
  }

  public static LinearVelocity angularToLinearVelocity(
      Distance radius, AngularVelocity angularVelocity) {
    return MetersPerSecond.of(radius.in(Meters) * angularVelocity.in(RadiansPerSecond));
  }

  public static AngularVelocity linearToAngularVelocity(
      Distance radius, LinearVelocity linearVelocity) {
    return RadiansPerSecond.of(linearVelocity.in(MetersPerSecond) / radius.in(Meters));
  }

  public static Rotation2d round(Rotation2d rotation) {
    return Rotation2d.fromRotations(Math.round(rotation.getRotations()));
  }

  public static Rotation2d floor(Rotation2d rotation) {
    return Rotation2d.fromRotations(Math.floor(rotation.getRotations()));
  }

  public static Rotation2d ceil(Rotation2d rotation) {
    return Rotation2d.fromRotations(Math.ceil(rotation.getRotations()));
  }

  public static Rotation2d abs(Rotation2d rotation) {
    return Rotation2d.fromRotations(Math.abs(rotation.getRotations()));
  }

  /** Returns the difference between two angles, in the range [-0.5, 0.5] */
  public static Angle minDifference(Angle a, Angle b) {
    return Rotations.of(MathUtils.floorMod(a.in(Rotations) - b.in(Rotations) - 0.5, 1) - 0.5);
  }

  public static Angle minAbsDifference(Angle a, Angle b) {
    return Rotations.of(
        Math.abs(MathUtils.floorMod(a.in(Rotations) - b.in(Rotations) - 0.5, 1) - 0.5));
  }

  public static Rotation2d minDifference(Rotation2d a, Rotation2d b) {
    return Rotation2d.fromRotations(
        MathUtils.floorMod(a.getRotations() - b.getRotations() - 0.5, 1) - 0.5);
  }

  public static Rotation2d minAbsDifference(Rotation2d a, Rotation2d b) {
    return MeasureMath.abs(minDifference(a, b));
  }

  public static Time time(double minutes, double seconds) {
    return Seconds.of(minutes * 60 + seconds);
  }

  public static Rotation2d fromMeasure(Angle angle) {
    return Rotation2d.fromRadians(angle.in(Radians));
  }

  public static Rotation2d fromMeasure(AngularVelocity velocity) {
    return Rotation2d.fromRadians(velocity.in(RadiansPerSecond));
  }

  public static Angle toAngle(Measure<AngleUnit> m) {
    return Rotations.of(m.in(Rotations));
  }

  public static AngularVelocity toAngularVelocity(Measure<AngularVelocityUnit> velocity) {
    return RotationsPerSecond.of(velocity.in(RotationsPerSecond));
  }

  public static Distance toDistance(Measure<DistanceUnit> distance) {
    return Meters.of(distance.in(Meters));
  }

  public static LinearVelocity toLinearVelocity(Measure<LinearVelocityUnit> distance) {
    return MetersPerSecond.of(distance.in(MetersPerSecond));
  }
}
