package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.pathplanner.lib.config.PIDConstants;
import com.team6962.lib.swerve.SwerveConfig;
import com.team6962.lib.swerve.SwerveConfig.Chassis;
import com.team6962.lib.swerve.SwerveConfig.DriveGains;
import com.team6962.lib.swerve.SwerveConfig.Gearing;
import com.team6962.lib.swerve.SwerveConfig.Module;
import com.team6962.lib.swerve.SwerveConfig.Motor;
import com.team6962.lib.swerve.SwerveConfig.Wheel;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Preferences;

public final class SwerveConstants {
  private SwerveConstants() {}

  private static SwerveConfig config;

  public static SwerveConfig get() {
    if (config == null) {
      config = generateConfiguration();
    }

    return config;
  }

  private static SwerveConfig generateConfiguration() {
    ChassisType chassisType = getChassisType();

    return new SwerveConfig(
            getChassis(chassisType),
            Gearing.MK4I_L2_PLUS,
            getModules(chassisType),
            new Motor(
                DCMotor.getKrakenX60(1),
                new Slot0Configs()
                    .withKP(0.1)
                    .withKI(0.01)
                    .withKD(0.01)
                    .withKV(0.118)
                    .withKA(0.0045)
                    .withKS(0.17)
                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign),
                Amps.of(250)),
            new Motor(
                DCMotor.getKrakenX60(1),
                new Slot0Configs()
                    .withKS(0.282)
                    .withKV(2.6)
                    .withKA(0.03)
                    .withKP(18.592)
                    .withKI(0.0)
                    .withKD(0.972)
                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign),
                Amps.of(60)),
            Wheel.BILLET.withDiameter(Inches.of(3.9052667792884215)),
            new DriveGains(new PIDConstants(0.5, 0.0, 0.1), new PIDConstants(0.25, 0.0, 0.05))
                .withFineTranslation(new PIDConstants(3.5, 0.0, 1.5))
                .withFineRotation(new PIDConstants(3.5, 0.0, 1.5)))
        .withMaxDriveSpeed(MetersPerSecond.of(4.474))
        .withMaxLinearAcceleration(MetersPerSecondPerSecond.of(6.579))
        .withMaxRotationSpeed(RotationsPerSecond.of(1.62))
        .withMaxAngularAcceleration(RotationsPerSecondPerSecond.of(1.28))
        .withCANBus(chassisType == ChassisType.COMPETITION ? "drivetrain" : "rio");
  }

  private static Chassis getChassis(ChassisType chassisType) {
    return switch (chassisType) {
      case COMPETITION -> new Chassis(
          Inches.of(36), Inches.of(36), Inches.of(24.75), Inches.of(24.75), Pounds.of(115));
      case TEST -> new Chassis(
          Inches.of(36), Inches.of(36), Inches.of(24.75), Inches.of(24.75), Pounds.of(135));
    };
  }

  private static enum ChassisType {
    COMPETITION,
    TEST
  }

  private static ChassisType getChassisType() {
    String idString = Preferences.getString("Chassis", "COMPETITION");

    if (idString.equals("TEST")) {
      System.out.println("=== !!! ### TEST CHASSIS ### !!! ===");
      return ChassisType.TEST;
    } else if (idString.equals("COMPETITION")) {
      System.out.println("=== COMPETITION CHASSIS ===");

      return ChassisType.COMPETITION;
    } else {
      System.out.println("Bad chassis id. Default to");
      System.out.println("=== COMPETITION CHASSIS ===");

      return ChassisType.COMPETITION;
    }
  }

  private static Module[] getModules(ChassisType chassisType) {
    return switch (chassisType) {
      case COMPETITION -> new Module[] {
        Constants.SWERVE.MODULE_CONFIGS[1],
        Constants.SWERVE.MODULE_CONFIGS[0],
        Constants.SWERVE.MODULE_CONFIGS[2],
        Constants.SWERVE.MODULE_CONFIGS[3]
      };
      case TEST -> new Module[] {
        Constants.SWERVE.MODULE_CONFIGS[5], // Front Left
        Constants.SWERVE.MODULE_CONFIGS[4], // Front Right
        Constants.SWERVE.MODULE_CONFIGS[7], // Back Left
        Constants.SWERVE.MODULE_CONFIGS[6] // Back Right
      };
    };
  }
}
