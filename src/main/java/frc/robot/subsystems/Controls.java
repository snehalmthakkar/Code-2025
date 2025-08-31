package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;

import com.team6962.lib.swerve.SwerveDrive;
import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.utils.MeasureMath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants.DEVICES;
import frc.robot.constants.Constants.ELEVATOR;
import frc.robot.constants.Constants.MANIPULATOR_PIVOT;
import frc.robot.auto.AutoAlign;
import frc.robot.auto.Autonomous;
import frc.robot.auto.AutoAlign.PolePattern;
import frc.robot.commands.PieceCombos;
import frc.robot.commands.XBoxSwerve;
import frc.robot.field.ReefPositioning;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.hang.Hang;
import frc.robot.subsystems.manipulator.Manipulator;

import java.util.Set;
import java.util.function.BooleanSupplier;

public class Controls {
  public static final CommandXboxController operator =
      new CommandXboxController(DEVICES.OPERATOR_XBOX_CONTROLLER);
  public static final CommandXboxController driver =
      new CommandXboxController(DEVICES.DRIVE_XBOX_CONTROLLER);

  public static void configureBindings(
      SwerveDrive swerveDrive,
      Elevator elevator,
      Manipulator manipulator,
      Hang hang,
      AutoAlign autoAlign,
      Autonomous autonomous,
      PieceCombos pieceCombos) {

    // Driver
    // Move swerve chassis
    // Rotate Swerve Chassis
    // ^^^ see xbox swerve.java

    // Button to move to processor
    // Button to move to source
    // Auto orient towards algae
    // Button to move to left/right reef (dpad left right)
    // Button for aligning to algae on the reef (dpad up)

    driver.a().whileTrue(autoAlign.alignToClosestFaceTeleop());
    driver
        .b()
        .whileTrue(
            autoAlign.alignToClosestPoleTeleop(
                AutoAlign.PolePattern.RIGHT,
                () ->
                    rumbleBoth()
                        .repeatedly()
                        .alongWith(LEDs.setStateCommand(LEDs.State.AUTO_ALIGN))));

    driver
        .x()
        .whileTrue(
            autoAlign.alignToClosestPoleTeleop(
                AutoAlign.PolePattern.LEFT,
                () ->
                    rumbleBoth()
                        .repeatedly()
                        .alongWith(LEDs.setStateCommand(LEDs.State.AUTO_ALIGN))));
    driver.y().whileTrue(autograbAlgae2(swerveDrive, elevator, manipulator, pieceCombos, autoAlign));
    driver.start().whileTrue(autoAlign.alignToClosestL1Teleop(
      AutoAlign.PolePattern.LEFT,
      () -> rumbleBoth()
        .repeatedly()
        .alongWith(LEDs.setStateCommand(LEDs.State.AUTO_ALIGN))
    ));
    driver.back().whileTrue(autoAlign.alignToClosestL1Teleop(
      AutoAlign.PolePattern.RIGHT,
      () -> rumbleBoth()
        .repeatedly()
        .alongWith(LEDs.setStateCommand(LEDs.State.AUTO_ALIGN))
    ));
    driver.leftBumper();
    driver.rightBumper();
    // driver.rightStick().onTrue(pieceCombos.pickupGroundAlgae());
    // driver.leftStick().onTrue(pieceCombos.algaeProcessor());
    driver
        .leftStick()
        .onTrue(pieceCombos.algaeProcessor()); // TODO: Change to whileTrue() before test
    driver.povCenter(); // USED
    driver.povUp(); // USED
    driver.povDown(); // USED
    driver.povLeft(); // USED
    driver.povRight(); // USED
    driver.leftTrigger(); // USED
    driver.rightTrigger(); // USED

    XboxController driverHID = driver.getHID();
    XboxController operatorHID = operator.getHID();

    Logger.logXBoxController("Controllers/Driver", driverHID);
    Logger.logXBoxController("Controllers/Operator", operatorHID);

    swerveDrive.setDefaultCommand(new XBoxSwerve(swerveDrive, driverHID));

    // Operator
    // Button to L2-L4, and Barge Height
    // Intake Algae with The Box
    // Output Algae from The Box
    // Output Coral
    // Deploy Hang
    // Retract Hang
    // L2 Algae Removal height
    // L3 Algae Removal Height
    // Algae ground Height

    operator.a().onTrue(pieceCombos.coralL1());
    operator.b().onTrue(autoscoreCoral(swerveDrive, elevator, manipulator, pieceCombos, 2));
    operator.x().onTrue(autoscoreCoral(swerveDrive, elevator, manipulator, pieceCombos, 3));
    operator.y().onTrue(autoscoreCoral(swerveDrive, elevator, manipulator, pieceCombos, 4));

    operator.povUp().whileTrue(elevator.up());
    operator.povDown().whileTrue(elevator.down());
    operator.povRight().whileTrue(manipulator.pivot.up());
    operator.povLeft().whileTrue(manipulator.pivot.down());
    operator.back().onTrue(pieceCombos.algaeL3());
    operator.start().onTrue(pieceCombos.algaeL2());
    operator
        .leftStick()
        .onTrue(
            pieceCombos.algaeBargeSetup().andThen(pieceCombos.algaeBargeShoot())); // barge combo
    operator
        .rightStick()
        .whileTrue(
            pieceCombos
                .intakeCoral()
                .andThen(
                    Commands.parallel(
                        rumbleBoth(),
                        LEDs.setStateCommand(LEDs.State.GOOD),
                        pieceCombos.coralL2()
                    ))); // big right paddle

    operator.rightBumper().whileTrue(manipulator.grabber.adjustCoral()); // intake coral
    operator
        .rightTrigger()
        .whileTrue(
            pieceCombos
                .intakeAlgaeOrShootCoral()
                .andThen(
                    rumbleBoth()
                        .alongWith(
                            LEDs.setStateCommand(LEDs.State.GOOD)))); // drop coral/intake algae
    operator.leftBumper().whileTrue(pieceCombos.algaeBargeShoot()); // shoot barge
    operator
        .leftTrigger()
        .whileTrue(
            manipulator
                .grabber
                .dropAlgae()
                .andThen(
                    LEDs.setStateCommand(LEDs.State.GOOD)) // ✅ Only runs when button is pressed
            );
  }

  private static boolean isNearCoralPole(SwerveDrive swerveDrive, Distance translationTolerance, Angle rotationTolerance, PolePattern pattern) {
    for (int i = pattern.start; i < 12; i += pattern.increment) {
      Pose2d placePose = ReefPositioning.getCoralPlacePose(i);

      if (swerveDrive.isWithinToleranceOf(placePose, translationTolerance, rotationTolerance)) {
        return true;
      }
    }

    return false;
  }

  private static Command autoscoreCoral(
      SwerveDrive swerveDrive,
      Elevator elevator,
      Manipulator manipulator,
      PieceCombos pieceCombos,
      int level) {
    return Commands.sequence(
      Commands.deadline(
        Commands.waitUntil(() -> {
          return (driver.getHID().getBButton() || driver.getHID().getXButton()) && isNearCoralPole(
              swerveDrive, Feet.of(2), Degrees.of(45),
              driver.getHID().getBButton() ? PolePattern.RIGHT : PolePattern.LEFT
            );
        }),
        level == 2 ? pieceCombos.coralL2() : pieceCombos.readyL3()
      ),
      Commands.parallel(
        pieceCombos.coral(level),
        Commands.waitUntil(
          () -> {
            if (level == 1) return false;

            if (!isNearCoralPole(swerveDrive, Inches.of(0.85), Degrees.of(4), PolePattern.ALL)) return false;

            Distance targetHeight =
                level == 2
                    ? ELEVATOR.CORAL.L2_HEIGHT
                    : level == 3 ? ELEVATOR.CORAL.L3_HEIGHT : ELEVATOR.CORAL.L4_HEIGHT;

            if (!elevator.inRange(targetHeight)) return false;

            Angle targetAngle =
                level == 2 || level == 3
                    ? MANIPULATOR_PIVOT.CORAL.L23_ANGLE
                    : MANIPULATOR_PIVOT.CORAL.L4_ANGLE;

            if (MeasureMath.minAbsDifference(manipulator.pivot.getAngle(), targetAngle)
                .gte(Degrees.of(2))) return false;

            return true;
          }
        )
      ),
      Commands.parallel(pieceCombos.coral(level), manipulator.grabber.dropCoral())
    );
  }

  public static Command autograbAlgae(
    SwerveDrive swerveDrive,
    Elevator elevator,
    Manipulator manipulator,
    PieceCombos pieceCombos,
    AutoAlign autoAlign
  ) {
    return Commands.defer(() -> {
      int face = autoAlign.getClosestReefFace(swerveDrive.getEstimatedPose());
      int level = ReefPositioning.getAlgaeHeight(face);

      return Commands.sequence(
        Commands.deadline(
          swerveDrive
            .pathfindTo(ReefPositioning.getAlgaeAlignPose(face))
            .andThen(swerveDrive.driveTo(ReefPositioning.getAlgaeAlignPose(face)))
              .until(() -> swerveDrive.isWithinToleranceOf(ReefPositioning.getAlgaeAlignPose(face), Inches.of(3), Degrees.of(10))),
          Commands.sequence(
            Commands.deadline(
              manipulator.pivot.stow().until(() -> manipulator.pivot.getAngle().gt(Degrees.of(-10))),
              elevator.hold()
            ),
            Commands.parallel(
              (level == 2 ? elevator.algaeL2() : elevator.ready()).andThen(elevator.hold()),
              manipulator.pivot.hold()
            )
          )
        ),
        manipulator.pivot.stow().until(() -> manipulator.pivot.getAngle().gt(Degrees.of(-10))).deadlineFor(elevator.hold()),
        Commands.deadline(
          level == 2 ? elevator.algaeL2() : elevator.algaeL3(),
          manipulator.pivot.hold()
        ),
        Commands.deadline(
          manipulator.grabber.intakeAlgae(),  
          manipulator.pivot.algaeReef().andThen(manipulator.pivot.hold()),
          swerveDrive.driveTo(ReefPositioning.getAlgaePickupPose(face)),
          elevator.hold()
        ),
        swerveDrive.driveTo(ReefPositioning.getAlgaeAlignPose(face))
          .deadlineFor(
            manipulator.pivot.pivotTo(() -> MANIPULATOR_PIVOT.ALGAE.HOLD_ANGLE),
            elevator.ready(),
            manipulator.grabber.holdAlgae().repeatedly()
          )
          .until(() -> swerveDrive.isWithinToleranceOf(ReefPositioning.getAlgaeAlignPose(face), Inches.of(3), Degrees.of(30))),
        Commands.runOnce(() -> manipulator.grabber.expectAlgae(true))
      );
    }, Set.of(swerveDrive.useRotation(), swerveDrive.useTranslation(), elevator, manipulator));
  }

  public static Command autograbAlgae2(
    SwerveDrive swerveDrive,
    Elevator elevator,
    Manipulator manipulator,
    PieceCombos pieceCombos,
    AutoAlign autoAlign
  ) {
    return Commands.defer(() -> {
      int face = autoAlign.getClosestReefFace(swerveDrive.getEstimatedPose());
      int level = ReefPositioning.getAlgaeHeight(face);

      return Commands.sequence(
        Commands.deadline(
          Commands.sequence(
            Commands.deadline(
              manipulator.pivot.stow().until(() -> manipulator.pivot.getAngle().gt(Degrees.of(-10))),
              elevator.hold(),
              manipulator.grabber.hold()
            )
          ),
          swerveDrive
            .pathfindTo(ReefPositioning.getAlgaeAlignPose(face))
            .andThen(swerveDrive.driveTo(ReefPositioning.getAlgaeAlignPose(face)))
        ),
        Commands.parallel(
          swerveDrive.driveTo(ReefPositioning.getAlgaePickupPose(face)),
          Commands.sequence(
            Commands.deadline(
              (level == 2 ? elevator.algaeL2() : elevator.ready()),
              manipulator.pivot.stow().andThen(manipulator.pivot.hold()),
              manipulator.grabber.hold()
            ),
            Commands.parallel(
              manipulator.grabber.intakeAlgae(),
              manipulator.pivot.algaeReef().andThen(manipulator.pivot.hold()),
              elevator.hold()
            )
          )
        ).until(manipulator.grabber::hasAlgae),
        swerveDrive.driveTo(ReefPositioning.getAlgaeAlignPose(face))
          .deadlineFor(
            manipulator.pivot.pivotTo(() -> MANIPULATOR_PIVOT.ALGAE.HOLD_ANGLE).andThen(manipulator.pivot.hold()),
            elevator.ready().andThen(elevator.hold()),
            manipulator.grabber.holdAlgae().repeatedly()
          )
          .until(() -> swerveDrive.isWithinToleranceOf(ReefPositioning.getAlgaeAlignPose(face), Inches.of(3), Degrees.of(30))),
        Commands.runOnce(() -> manipulator.grabber.expectAlgae(true))
      );
    }, Set.of(swerveDrive.useRotation(), swerveDrive.useTranslation(), elevator, manipulator));
  }

  private static Command rumble(CommandXboxController controller) {
    return Commands.runEnd(
            () -> {
              controller.getHID().setRumble(RumbleType.kBothRumble, 1.0);
            },
            () -> {
              controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);
            })
        .withTimeout(0.25);
  }

  private static Command rumble(CommandXboxController controller, BooleanSupplier booleanSupplier) {
    return Commands.runEnd(
        () -> {
          if (booleanSupplier.getAsBoolean()) {
            controller.getHID().setRumble(RumbleType.kBothRumble, 1.0);
          } else {
            controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);
          }
        },
        () -> {
          controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);
        });
  }

  public static Command rumbleDriver() {
    return rumble(driver);
  }

  public static Command rumbleDriver(BooleanSupplier booleanSupplier) {
    return rumble(driver, booleanSupplier);
  }

  public static Command rumbleOperator() {
    return rumble(operator);
  }

  public static Command rumbleOperator(BooleanSupplier booleanSupplier) {
    return rumble(operator, booleanSupplier);
  }

  public static Command rumbleBoth() {
    return rumbleOperator().alongWith(rumbleDriver());
  }

  public static Command rumbleBoth(BooleanSupplier booleanSupplier) {
    return rumbleOperator(booleanSupplier).alongWith(rumbleDriver(booleanSupplier));
  }
}
