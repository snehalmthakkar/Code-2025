package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.subsystems.manipulator.grabber.Grabber;

/**
 * SimElevator is a simulation implementation of the Elevator subsystem.
 * This class provides simulation-specific logic for elevator motion,
 * sensor feedback, motor voltage application, and limit switch simulation.
 */
public class SimElevator extends Elevator {
    // Simulated state for the left elevator motor.
    private TalonFXSimState leftSimState = new TalonFXSimState(leftMotor);

    // Simulated state for the right elevator motor.
    private TalonFXSimState rightSimState = new TalonFXSimState(rightMotor);

    // WPILib ElevatorSim models the elevator's physics and sensor feedback.
    private ElevatorSim elevatorSim = new ElevatorSim(
        LinearSystemId.createElevatorSystem(
            DCMotor.getKrakenX60Foc(2),
            ElevatorConstants.ELEVATOR_MASS.in(Kilograms),
            ElevatorConstants.SPOOL_DIAMETER.in(Meters) / 2.0,
            ElevatorConstants.MOTOR_GEAR_REDUCTION
        ),
        DCMotor.getKrakenX60Foc(2),
        ElevatorConstants.MIN_HEIGHT.in(Meters),
        ElevatorConstants.MAX_HEIGHT.in(Meters),
        true,
        ElevatorConstants.MIN_HEIGHT.in(Meters)
    );

    /**
     * Constructor for the SimElevator, passing the grabber object to the superclass.
     *
     * @param grabber The grabber subsystem used for inter-subsystem coordination.
     */
    public SimElevator(Grabber grabber) {
        super(grabber);
    }
    
    /**
     * Checks if the simulated top limit switch is triggered,
     * based on the position of the elevator in meters.
     *
     * @return true if at or just below the max height, otherwise false
     */
    @Override
    protected boolean topLimitSwitchTriggered() {
        return elevatorSim.getPositionMeters() >= ElevatorConstants.MAX_HEIGHT.in(Meters) - 0.001;
    }

    /**
     * Checks if the simulated bottom limit switch is triggered,
     * based on the position of the elevator in meters.
     *
     * @return true if at or just above the min height, otherwise false
     */
    @Override
    protected boolean bottomLimitSwitchTriggered() {
        return elevatorSim.getPositionMeters() <= ElevatorConstants.MIN_HEIGHT.in(Meters) + 0.001;
    }

    /**
     * Gets the applied voltage for the left motor, accounting for inversion direction.
     *
     * @return The effective voltage for the left motor.
     */
    private double getLeftMotorVoltage() {
        return (ElevatorConstants.LEFT_MOTOR_INVERTED_VALUE == InvertedValue.Clockwise_Positive ? -1 : 1) * leftSimState.getMotorVoltage();
    }

    /**
     * Gets the applied voltage for the right motor, accounting for inversion direction.
     *
     * @return The effective voltage for the right motor.
     */
    private double getRightMotorVoltage() {
        return (ElevatorConstants.RIGHT_MOTOR_INVERTED_VALUE == InvertedValue.Clockwise_Positive ? -1 : 1) * rightSimState.getMotorVoltage();
    }

    /**
     * Sets the raw rotor position for the left motor in simulation.
     *
     * @param angle Raw angle to set, adjusted for inversion.
     */
    private void setLeftRotorPosition(double angle) {
        leftSimState.setRawRotorPosition(angle * (ElevatorConstants.RIGHT_MOTOR_INVERTED_VALUE == InvertedValue.Clockwise_Positive ? -1 : 1));
    }

    /**
     * Sets the raw rotor position for the right motor in simulation.
     *
     * @param angle Raw angle to set, adjusted for inversion.
     */
    private void setRightRotorPosition(double angle) {
        rightSimState.setRawRotorPosition(angle * (ElevatorConstants.RIGHT_MOTOR_INVERTED_VALUE == InvertedValue.Clockwise_Positive ? -1 : 1));
    }

    /**
     * Sets the rotor velocity for the left motor in simulation.
     *
     * @param velocity Rotor velocity to set, adjusted for inversion.
     */
    private void setLeftRotorVelocity(double velocity) {
        leftSimState.setRotorVelocity(velocity * (ElevatorConstants.RIGHT_MOTOR_INVERTED_VALUE == InvertedValue.Clockwise_Positive ? -1 : 1));
    }

    /**
     * Sets the rotor velocity for the right motor in simulation.
     *
     * @param velocity Rotor velocity to set, adjusted for inversion.
     */
    private void setRightRotorVelocity(double velocity) {
        rightSimState.setRotorVelocity(velocity * (ElevatorConstants.RIGHT_MOTOR_INVERTED_VALUE == InvertedValue.Clockwise_Positive ? -1 : 1));
    }

    /**
     * This method is called every simulation cycle (typically every 20ms).
     * It updates the ElevatorSim input voltage, steps the physics simulation, updates sensors,
     * sets simulated supply voltages, and synchronizes the simulated motor state.
     */
    @Override
    public void simulationPeriodic() {
        // Set input voltage for elevator physics simulation.
        elevatorSim.setInputVoltage((getLeftMotorVoltage() + getRightMotorVoltage()) / 2);

        // Step simulation forward by 20ms.
        elevatorSim.update(0.02);
        
        // Get position & velocity from the simulation.
        // double sprocketCircumference = Math.PI * NewElevatorConstants.SPOOL_DIAMETER.in(Meters);
        double position = elevatorSim.getPositionMeters();
        double velocity = elevatorSim.getVelocityMetersPerSecond();
        double motorAngle = position * ElevatorConstants.SENSOR_MECHANISM_RATIO;
        double motorVelocity = velocity * ElevatorConstants.SENSOR_MECHANISM_RATIO;

        // Synchronize simulated motor rotor positions and velocities with elevator mechanism.
        setLeftRotorPosition(motorAngle);
        setRightRotorPosition(motorAngle);

        setLeftRotorVelocity(motorVelocity);
        setRightRotorVelocity(motorVelocity);

        // Update simulated supply voltages to match the robot's battery voltage.
        leftSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        rightSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    }
}
