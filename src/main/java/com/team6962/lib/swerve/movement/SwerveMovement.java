package com.team6962.lib.swerve.movement;

import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.team6962.lib.swerve.SwerveCore;

public interface SwerveMovement {
  public static NeutralOut neutralOut = new NeutralOut();
  public static PositionVoltage positionVoltage = new PositionVoltage(0);
  public static VelocityVoltage velocityVoltage = new VelocityVoltage(0);
  public static MotionMagicExpoVoltage motionMagicExpoVoltage = new MotionMagicExpoVoltage(0);
  public static MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);
  public static DynamicMotionMagicVoltage dynamicMotionMagicVoltage =
      new DynamicMotionMagicVoltage(0, 0, 0, 0);
  public static MotionMagicVelocityVoltage motionMagicVelocityVoltage =
      new MotionMagicVelocityVoltage(0);

  public default void log() {}

  public void execute(SwerveCore drivetrain);

  public SwerveMovement cleared();
}
