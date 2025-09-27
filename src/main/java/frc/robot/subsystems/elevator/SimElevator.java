package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class SimElevator extends Elevator {
    private TalonFXSimState leftSimState = new TalonFXSimState(leftMotor);
    private TalonFXSimState rightSimState = new TalonFXSimState(rightMotor);

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

    public SimElevator() {
        super();
    }
    
    @Override
    protected boolean topLimitSwitchTriggered() {
        return elevatorSim.getPositionMeters() >= ElevatorConstants.MAX_HEIGHT.in(Meters) - 0.001;
    }

    @Override
    protected boolean bottomLimitSwitchTriggered() {
        return elevatorSim.getPositionMeters() <= ElevatorConstants.MIN_HEIGHT.in(Meters) + 0.001;
    }

    private double getLeftMotorVoltage() {
        return (ElevatorConstants.LEFT_MOTOR_INVERTED_VALUE == InvertedValue.Clockwise_Positive ? -1 : 1) * leftSimState.getMotorVoltage();
    }

    private double getRightMotorVoltage() {
        return (ElevatorConstants.RIGHT_MOTOR_INVERTED_VALUE == InvertedValue.Clockwise_Positive ? -1 : 1) * rightSimState.getMotorVoltage();
    }

    private void setLeftRotorPosition(double angle) {
        leftSimState.setRawRotorPosition(angle * (ElevatorConstants.RIGHT_MOTOR_INVERTED_VALUE == InvertedValue.Clockwise_Positive ? -1 : 1));
    }

    private void setRightRotorPosition(double angle) {
        rightSimState.setRawRotorPosition(angle * (ElevatorConstants.RIGHT_MOTOR_INVERTED_VALUE == InvertedValue.Clockwise_Positive ? -1 : 1));
    }

    private void setLeftRotorVelocity(double velocity) {
        leftSimState.setRotorVelocity(velocity * (ElevatorConstants.RIGHT_MOTOR_INVERTED_VALUE == InvertedValue.Clockwise_Positive ? -1 : 1));
    }

    private void setRightRotorVelocity(double velocity) {
        rightSimState.setRotorVelocity(velocity * (ElevatorConstants.RIGHT_MOTOR_INVERTED_VALUE == InvertedValue.Clockwise_Positive ? -1 : 1));
    }

    @Override
    public void simulationPeriodic() {
        elevatorSim.setInputVoltage((getLeftMotorVoltage() + getRightMotorVoltage()) / 2);
        elevatorSim.update(0.02);
        
        // double sprocketCircumference = Math.PI * NewElevatorConstants.SPOOL_DIAMETER.in(Meters);
        double position = elevatorSim.getPositionMeters();
        double velocity = elevatorSim.getVelocityMetersPerSecond();
        double motorAngle = position * ElevatorConstants.SENSOR_MECHANISM_RATIO;
        double motorVelocity = velocity * ElevatorConstants.SENSOR_MECHANISM_RATIO;

        setLeftRotorPosition(motorAngle);
        setRightRotorPosition(motorAngle);

        setLeftRotorVelocity(motorVelocity);
        setRightRotorVelocity(motorVelocity);

        leftSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        rightSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    }
}
