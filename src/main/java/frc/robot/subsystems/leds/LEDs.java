// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.*;

import com.team6962.lib.telemetry.Logger;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.LED;
import frc.robot.util.CachedRobotState;
import frc.robot.vision.AprilTags;

public class LEDs extends SubsystemBase {
  private static AddressableLED strip;
  private static AddressableLEDBuffer buffer;
  private static State state = State.OFF;
  private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);
  private static final Distance kLedSpacing = Meters.of(1 / 20.0);
  private final LEDPattern m_scrollingRainbow =
      m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);

  public static enum State {
    OFF,
    DISABLED,
    DEFAULT,
    AUTO_RED, // Updated
    AUTO_BLUE, // Updated
    TELEOP_RED, // Updated
    TELEOP_BLUE, // Updated
    AUTO_ALIGN, // Updated
    GOOD
  }

  public static final Color WHITE = new Color(255, 255, 255);
  public static final Color ANTARES_BLUE = new Color(37, 46, 69);
  public static final Color ANTARES_BLUE_BRIGHT = new Color(37. / 69. * 0.5, 46. / 69. * 0.5, 0.5);
  public static final Color ANTARES_YELLOW = new Color(1., 222. / 242. * 0.93, 139. / 242.);
  public static final Color RED = new Color(255, 0, 0);
  public static final Color GREEN = new Color(0, 255, 1);
  public static final Color BLUE = new Color(0, 150, 200);
  public static final Color RSL_ORANGE = new Color(255, 100, 0);
  public static final Color LIGHT_BLUE = new Color(173, 216, 230);
  public static final Color YELLOW = new Color(255, 255, 0);
  public static final Color ORANGE = new Color(255, 100, 0);
  public static final Color CYAN = new Color(0, 255, 255);
  public static final Color DARK_GREEN = new Color(0, 100, 0);
  public static final Color PURPLE = new Color(108, 59, 170);
  public static final Color MAGENTA = new Color(255, 0, 255);

  public LEDs() {
    strip = new AddressableLED(LED.port);
    buffer = new AddressableLEDBuffer(LED.SIDE_STRIP_HEIGHT);
    strip.setLength(buffer.getLength());

    strip.setData(buffer);
    strip.start();
  }

  public static Command setStateCommand(State state) {
    return Commands.run(() -> setState(state));
  }

  public static void setState(State state) {
    if (state.ordinal() > LEDs.state.ordinal()) LEDs.state = state;
  }

  /*public static void rainbowCommand(State state) {
    return Commands.run(() --> setState())
  }*/

  private static Color convertVisibleColorToDriverColor(Color visibleColor) {
    return new Color(visibleColor.green, visibleColor.red, visibleColor.blue);
  }

  private static LEDPattern createDiscontinuousGradient(
      Color from, Color to, Frequency scrollSpeed) {
    return LEDPattern.gradient(
      LEDPattern.GradientType.kDiscontinuous,
      convertVisibleColorToDriverColor(from),
      convertVisibleColorToDriverColor(to)
    ).scrollAtRelativeSpeed(scrollSpeed);
  }

  private static LEDPattern createContinuousGradient(
      Color from, Color to, Frequency scrollSpeed) {
    return LEDPattern.gradient(
      LEDPattern.GradientType.kContinuous,
      convertVisibleColorToDriverColor(from),
      convertVisibleColorToDriverColor(to)
    ).scrollAtRelativeSpeed(scrollSpeed);
  }

  private static void apply(LEDPattern pattern) {
    // Apply the LED pattern to the data buffer
    pattern.applyTo(buffer);

    // Write the data to the LED strip
    strip.setData(buffer);
  }

  @Override
  public void periodic() {
    switch (state) {
      case OFF:
        apply(createDiscontinuousGradient(new Color(0, 0, 0), new Color(0, 0, 0), Hertz.of(0)));
        break;
      case DISABLED:
        apply(createDiscontinuousGradient(RSL_ORANGE, RSL_ORANGE, Hertz.of(0.5)));
        break;
      case DEFAULT:
        m_scrollingRainbow.applyTo(buffer);
        strip.setData(buffer);
        break;
      case AUTO_RED:
        apply(createDiscontinuousGradient(new Color(255, 100, 0), ANTARES_YELLOW, Hertz.of(2)));
        break;
      case AUTO_BLUE:
        apply(createDiscontinuousGradient(new Color(0, 255, 100), new Color(0, 255, 255), Hertz.of(2)));
        break;
      case TELEOP_RED:
        apply(createDiscontinuousGradient(new Color(1., 0.35, 0.1), new Color(1., 222. / 242. * 0.8, 139. / 242.), Hertz.of(0.5)));
        break;
      case TELEOP_BLUE:
        apply(createDiscontinuousGradient(ANTARES_BLUE_BRIGHT, BLUE, Hertz.of(1.25)));
        break;
      case AUTO_ALIGN:
        apply(createContinuousGradient(ANTARES_YELLOW, ORANGE, Hertz.of(1)));
        break;
      case GOOD:
        apply(createContinuousGradient(new Color(125, 0, 255), new Color(255, 75, 125), Hertz.of(1)));
        break;

    }

    if (CachedRobotState.isDisabled() && AprilTags.changingHeading) {
      state = State.DEFAULT;
    } else {
      state = (CachedRobotState.isAutonomous() && CachedRobotState.isEnabled()) ? (
        CachedRobotState.isBlue().orElse(false)
          ? State.AUTO_BLUE
          : State.AUTO_RED
      ) : (
        CachedRobotState.isBlue().orElse(false)
          ? State.TELEOP_BLUE
          : State.TELEOP_RED
      );
    }
  }
}
