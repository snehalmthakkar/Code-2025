package com.team6962.lib.swerve.module;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.junit.jupiter.api.Assertions.assertEquals;

import com.team6962.lib.swerve.SwerveConfig;
import com.team6962.lib.swerve.SwerveConfig.Gearing;
import com.team6962.lib.swerve.SwerveConfig.Wheel;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import org.junit.jupiter.api.Test;

public class GearingConversionTest {
  private SwerveConfig config =
      new SwerveConfig(null, Gearing.MK4I_L2_PLUS, null, null, null, Wheel.COLSON, null, null);

  @Test
  public void driveMotorRotorToMechanismPosition() {
    Angle drivePosition = Radians.of(1.23);
    Angle gearedPosition = drivePosition.div(5.92);
    Distance groundPosition = Inches.of(2).times(gearedPosition.in(Radians));

    assertEquals(groundPosition, config.driveMotorRotorToMechanism(drivePosition));
  }

  @Test
  public void driveMotorPositionConsistent() {
    Distance drivePosition = Meters.of(1.23);

    assertEquals(
        drivePosition,
        config.driveMotorRotorToMechanism(config.driveMotorMechanismToRotor(drivePosition)));
  }

  @Test
  public void driveMotorRotorToMechanismVelocity() {
    AngularVelocity driveVelocity = RadiansPerSecond.of(1.23);
    AngularVelocity gearedVelocity = driveVelocity.div(5.92);
    LinearVelocity groundVelocity =
        InchesPerSecond.of(2).times(gearedVelocity.in(RadiansPerSecond));

    assertEquals(groundVelocity, config.driveMotorRotorToMechanism(driveVelocity));
  }

  @Test
  public void driveMotorVelocityConsistent() {
    LinearVelocity driveVelocity = MetersPerSecond.of(85.6);

    assertEquals(
        driveVelocity,
        config.driveMotorRotorToMechanism(config.driveMotorMechanismToRotor(driveVelocity)));
  }

  @Test
  public void steerMotorRotorToMechanismPosition() {
    Angle steerPosition = Radians.of(1.23);
    Angle wheelPosition = steerPosition.div(150.0 / 7.0);

    assertEquals(wheelPosition, config.steerMotorRotorToMechanism(steerPosition));
  }

  @Test
  public void steerMotorPositionConsistent() {
    Angle steerPosition = Radians.of(2.42);

    assertEquals(
        steerPosition,
        config.steerMotorRotorToMechanism(config.steerMotorMechanismToRotor(steerPosition)));
  }

  @Test
  public void steerMotorRotorToMechanismVelocity() {
    AngularVelocity steerVelocity = RadiansPerSecond.of(1.23);
    AngularVelocity wheelVelocity = steerVelocity.div(150.0 / 7.0);

    assertEquals(wheelVelocity, config.steerMotorRotorToMechanism(steerVelocity));
  }

  @Test
  public void steerMotorVelocityConsistent() {
    AngularVelocity steerVelocity = RadiansPerSecond.of(-1.3);

    assertEquals(
        steerVelocity,
        config.steerMotorRotorToMechanism(config.steerMotorMechanismToRotor(steerVelocity)));
  }
}
