package frc.robot.subsystems.intake.rollers;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team6962.lib.utils.CTREUtils;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.intake.IntakeConstants;

/**
 * A simulation of the intake rollers subsystem, which pulls coral off of the
 * ground and into the robot. This class extends the {@link IntakeRollers}
 * class, which represents a real intake rollers subsystem.
 */
public class IntakeRollersSim extends IntakeRollers {
    private DCMotorSim physicsSim;
    private TalonFXSimState controllerSim;
    private Timer deltaTimer;

    /**
     * Creates a new IntakeRollersSim.
     */
    public IntakeRollersSim() {
        super();

        physicsSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                IntakeConstants.Simulation.rollersMotor,
                IntakeConstants.Simulation.rollersMOI.in(KilogramSquareMeters),
                IntakeConstants.Simulation.rollersReduction
            ),
            IntakeConstants.Simulation.rollersMotor
        );

        controllerSim = new TalonFXSimState(motor);
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

        double gearing = IntakeConstants.Simulation.rollersReduction;

        CTREUtils.check(controllerSim.setRawRotorPosition(physicsSim.getAngularPosition().times(gearing)));
        CTREUtils.check(controllerSim.setRotorVelocity(physicsSim.getAngularVelocity().times(gearing)));
        CTREUtils.check(controllerSim.setRotorAcceleration(physicsSim.getAngularAcceleration().times(gearing)));
    }
}
