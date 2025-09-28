package frc.robot.subsystems.manipulator.grabber;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import com.team6962.lib.utils.CTREUtils;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSensor extends SubsystemBase {
    private final CANrange canRange;

    private final StatusSignal<Distance> distanceSignal;
    private Distance distance = Meters.of(0);

    public AlgaeSensor(int deviceId, String canBus) {
        canRange = new CANrange(deviceId, canBus);
        
        CANrangeConfiguration canRangeConfig = new CANrangeConfiguration()
            .withFovParams(
                new FovParamsConfigs()
                    .withFOVRangeX(Degrees.of(6.75))
                    .withFOVRangeY(Degrees.of(6.75))
            );
        
        CTREUtils.check(canRange.getConfigurator().apply(canRangeConfig));
        
        distanceSignal = canRange.getDistance();
    }

    public Distance getDistance() {
        return distance;
    }

    public boolean hasAlgae() {
        return distance.lt(Inches.of(3.0));
    }

    public boolean isAlgaeFullyIntaked() {
        return distance.lt(Inches.of(0.25));
    }

    @Override
    public void periodic() {
        distance = CTREUtils.unwrap(distanceSignal.refresh());
    }
}
