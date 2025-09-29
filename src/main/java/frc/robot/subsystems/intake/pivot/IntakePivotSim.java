package frc.robot.subsystems.intake.pivot;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team6962.lib.utils.CTREUtils;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSensors;

/**
 * A simulation of the intake pivot, which pivots the ground coral intake up and
 * down. This class extends the {@link IntakePivot} class, which represents a
 * real intake pivot.
 */
public class IntakePivotSim extends IntakePivot {
    private SingleJointedArmSim physicsSim;
    private TalonFXSimState controllerSim;
    private CANcoderSimState encoderSim;

    private Timer deltaTimer;

    /**
     * Creates a new IntakePivotSim.
     */
    public IntakePivotSim(IntakeSensors sensors) {
        super(sensors);

        physicsSim = new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                IntakeConstants.Simulation.pivotMotor,
                IntakeConstants.Simulation.pivotMomentOfInertia.in(KilogramSquareMeters),
                IntakeConstants.pivotRotorToSensor * IntakeConstants.pivotSensorToMechanism
            ),
            IntakeConstants.Simulation.pivotMotor,
            IntakeConstants.pivotRotorToSensor * IntakeConstants.pivotSensorToMechanism,
            IntakeConstants.Simulation.pivotCenterOfMassDistance.in(Meters),
            IntakeConstants.pivotMinAngle.plus(IntakeConstants.centerOfMassAngularOffset).in(Radians),
            IntakeConstants.pivotMaxAngle.plus(IntakeConstants.centerOfMassAngularOffset).in(Radians),
            true,
            IntakeConstants.Simulation.pivotStartAngle.plus(IntakeConstants.centerOfMassAngularOffset).in(Radians)
        );

        controllerSim = new TalonFXSimState(motor);
        encoderSim = new CANcoderSimState(encoder);
    }

    @Override
    public TalonFXConfiguration modifyConfiguration(TalonFXConfiguration config) {
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        return config;
    }

    @Override
    protected CANcoderConfiguration modifyConfiguration(CANcoderConfiguration config) {
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        return config;
    }

    @Override
    public void periodic() {
        super.periodic();

        if (deltaTimer == null) {
            deltaTimer = new Timer();
            deltaTimer.start();
            return;
        }

        double dtSeconds = deltaTimer.get();
        deltaTimer.restart();

        CTREUtils.check(controllerSim.setSupplyVoltage(RobotController.getBatteryVoltage()));
        physicsSim.setInputVoltage(controllerSim.getMotorVoltage());

        physicsSim.update(dtSeconds);

        Angle physicsAngle = Radians.of(physicsSim.getAngleRads());
        Angle externalAngle = physicsAngle.minus(IntakeConstants.centerOfMassAngularOffset);
        Angle motorAngle = externalAngle.minus(IntakeConstants.absoluteEncoderOffset);

        CTREUtils.check(controllerSim.setRawRotorPosition(motorAngle.times(IntakeConstants.pivotRotorToSensor * IntakeConstants.pivotSensorToMechanism)));
        CTREUtils.check(controllerSim.setRotorVelocity(RadiansPerSecond.of(physicsSim.getVelocityRadPerSec()).times(IntakeConstants.pivotRotorToSensor * IntakeConstants.pivotSensorToMechanism)));
        
        CTREUtils.check(encoderSim.setRawPosition(motorAngle.times(IntakeConstants.pivotSensorToMechanism)));
        CTREUtils.check(encoderSim.setVelocity(RadiansPerSecond.of(physicsSim.getVelocityRadPerSec()).times(IntakeConstants.pivotSensorToMechanism)));
    }
}
