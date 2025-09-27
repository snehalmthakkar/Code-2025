package com.team6962.lib.swerve.auto;

import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.team6962.lib.swerve.SwerveConfig;
import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.utils.CTREUtils;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class SwerveGyroscope extends SubsystemBase {
    public SwerveGyroscope() {
        setName("Swerve Drive/Gyroscope");

        Logger.logMeasure(getName() + "/heading", this::getHeading);
    }

    public abstract Angle getHeading();

    public static SwerveGyroscope get(Supplier<SwerveModulePosition[]> moduleDeltasSupplier, SwerveDriveKinematics kinematics, SwerveConfig config) {
        if (RobotBase.isReal()) return new Pigeon(config);
        else return new Simulated(moduleDeltasSupplier, kinematics);
    }

    private static class Pigeon extends SwerveGyroscope {
        private final Pigeon2 pigeon;
        private final StatusSignal<Angle> headingSignal;

        public Pigeon(SwerveConfig config) {
            pigeon = new Pigeon2(config.gyroscope().canId(), config.canBus());

            Pigeon2Configuration pigeonConfig = new Pigeon2Configuration();
            pigeonConfig.MountPose = config.gyroscope().mountPose();
            CTREUtils.check(pigeon.getConfigurator().apply(pigeonConfig));

            headingSignal = pigeon.getYaw();
        }

        @Override
        public void periodic() {
            headingSignal.refresh();
        }

        @Override
        public Angle getHeading() {
            return CTREUtils.unwrap(headingSignal);
        }
    }

    private static class Simulated extends SwerveGyroscope {
        private Angle heading = Radians.of(0);

        private final Supplier<SwerveModulePosition[]> moduleDeltasSupplier;
        private final SwerveDriveKinematics kinematics;

        public Simulated(Supplier<SwerveModulePosition[]> moduleDeltasSupplier, SwerveDriveKinematics kinematics) {
            this.moduleDeltasSupplier = moduleDeltasSupplier;
            this.kinematics = kinematics;
        }

        @Override
        public Angle getHeading() {
            return heading;
        }

        @Override
        public void periodic() {
            Angle headingChange = Radians.of(kinematics.toTwist2d(moduleDeltasSupplier.get()).dtheta);

            if (Math.abs(headingChange.in(Radians)) < 0.01) {
                headingChange = Radians.of(0);
            }

            heading = heading.plus(headingChange);
        }
    }
}
