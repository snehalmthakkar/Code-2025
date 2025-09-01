// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.*;

import com.team6962.lib.telemetry.Logger;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
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
  private static State state = State.DEFAULT;
  private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);
  private static final Distance kLedSpacing = Meters.of(1 / 20.0);
  private final LEDPattern m_scrollingRainbow =
      m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);

  public static enum State {
    OFF,
    DISABLED,
    DEFAULT, // Right colors, fix scrolling
    DRIVING_AUTO, // good
    DRIVING_TELEOP_RED, // Wrong colors, fix scrolling
    DRIVING_TELEOP_BLUE, // Right colors, fix scroll
    AUTO_ALIGN,
    HAS_CORAL,
    GOOD,
    CAN_SEE_ALGAE
  }

  public static final Color WHITE = new Color(255, 255, 255);
  public static final Color ANTARES_BLUE = new Color(37, 46, 69);
  public static final Color ANTARES_YELLOW = new Color(222, 242, 139);
  public static final Color RED = new Color(0, 255, 0);
  public static final Color GREEN = new Color(255, 0, 1);
  public static final Color BLUE = new Color(0, 20, 255);
  public static final Color RSL_ORANGE = new Color(255, 100, 0);
  public static final Color LIGHT_BLUE = new Color(173, 216, 230);
  public static final Color YELLOW = new Color(255, 255, 0);
  public static final Color ORANGE = new Color(150, 255, 0);
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

  private static LEDPattern createColor(
      Color ColorFrom, Color ColorTo, double Blink, double Scroll) {
    LEDPattern base =
        LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, ColorFrom, ColorTo);
    LEDPattern blink = base.blink(Seconds.of(Blink));
    LEDPattern scroll = base.scrollAtRelativeSpeed(Percent.per(Second).of(Scroll));
    LEDPattern pattern = blink.overlayOn(scroll);

    return pattern;
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
        apply(createColor(new Color(0, 0, 0), new Color(0, 0, 0), 1.0, 0.0));
        break;
      case DISABLED:
        apply(createColor(RSL_ORANGE, RSL_ORANGE, 1.0, 50.0));
        break;
      case DEFAULT:
        m_scrollingRainbow.applyTo(buffer);
        strip.setData(buffer);
        break;
      case DRIVING_AUTO:
        apply(createColor(WHITE, WHITE, 1.0, 50.0));
        break;
      case DRIVING_TELEOP_RED:
        apply(createColor(RED, ANTARES_YELLOW, 1.0, 50.0));
        break;
      case DRIVING_TELEOP_BLUE:
        apply(createColor(BLUE, ANTARES_BLUE, 1.0, 50.0));
        break;
      case AUTO_ALIGN:
        apply(createColor(ANTARES_YELLOW, ORANGE, 1.0, 50.0));
        break;
      case HAS_CORAL:
        apply(createColor(ANTARES_YELLOW, ANTARES_YELLOW, 1.0, 50.0));
        break;
      case GOOD:
        apply(createColor(GREEN, GREEN, 1.0, 50.0));
        break;
      case CAN_SEE_ALGAE:
        apply(createColor(ANTARES_BLUE, CYAN, 1.0, 50.0));
        break;
        /*case HAS_ALGAE:
            apply(createColor(DARK_GREEN, DARK_GREEN, 1.0, 50.0));
            break;
          case HAS_CORAL:
            break;
          case CAN_SEE_ALGAE:
            apply(createColor(CYAN, CYAN, 1.0, 0.0));
            break;
          case AIMING_PROCESSOR:
            apply(createColor(DARK_GREEN, DARK_GREEN, 0.5, 0.0));
            break;
          case SCORING_PROCESSOR:
            apply(createColor(DARK_GREEN, DARK_GREEN, 1.0, 0.0));
            break;
          case AIMING_BARGE:
            apply(createColor(YELLOW, YELLOW, 0.5, 0.0));
            break;
          case SCORING_BARGE:
            apply(createColor(YELLOW, YELLOW, 1.0, 0.0));
            break;
          case AIMING_REEF:
            apply(createColor(MAGENTA, MAGENTA, 0.5, 0.0));
            break;
          case SCORING_REEF:
            apply(createColor(MAGENTA, MAGENTA, 0.0, 0.0));
            break;
        case HANG:
            apply(createColor(MAGENTA, MAGENTA, 1.0, 0.0));
            break;*/

    }

    Logger.log("changingHeading", AprilTags.changingHeading);

    if (CachedRobotState.isDisabled() && AprilTags.changingHeading) {
      state = State.DEFAULT;
    } else {
      state =
          CachedRobotState.isBlue().orElse(false)
              ? State.DRIVING_TELEOP_BLUE
              : State.DRIVING_TELEOP_RED;
    }
  }
}
