package frc.robot.subsystems.newelevator;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class SimElevator extends NewElevator {
    private TalonFXSimState leftSimState = new TalonFXSimState(leftMotor);
    private TalonFXSimState rightSimState = new TalonFXSimState(rightMotor);

    private ElevatorSim elevatorSim = new ElevatorSim(
        LinearSystemId.createElevatorSystem(
            DCMotor.getKrakenX60Foc(2),
            NewElevatorConstants.ELEVATOR_MASS.in(Kilograms),
            NewElevatorConstants.SPOOL_DIAMETER.in(Meters) / 2.0,
            NewElevatorConstants.MOTOR_GEAR_REDUCTION
        ),
        DCMotor.getKrakenX60Foc(2),
        NewElevatorConstants.MIN_HEIGHT.in(Meters),
        NewElevatorConstants.MAX_HEIGHT.in(Meters),
        true,
        NewElevatorConstants.MIN_HEIGHT.in(Meters)
    );

    public SimElevator() {
        super();
    }
    
    @Override
    protected boolean topLimitSwitchTriggered() {
        return elevatorSim.getPositionMeters() >= NewElevatorConstants.MAX_HEIGHT.in(Meters) - 0.01;
    }

    @Override
    protected boolean bottomLimitSwitchTriggered() {
        return elevatorSim.getPositionMeters() <= NewElevatorConstants.MIN_HEIGHT.in(Meters) + 0.01;
    }

    private double getLeftMotorVoltage() {
        return (NewElevatorConstants.LEFT_MOTOR_INVERTED_VALUE == InvertedValue.Clockwise_Positive ? -1 : 1) * leftSimState.getMotorVoltage();
    }

    private double getRightMotorVoltage() {
        return (NewElevatorConstants.RIGHT_MOTOR_INVERTED_VALUE == InvertedValue.Clockwise_Positive ? -1 : 1) * rightSimState.getMotorVoltage();
    }

    private void setLeftRotorPosition(double angle) {
        leftSimState.setRawRotorPosition(angle * (NewElevatorConstants.RIGHT_MOTOR_INVERTED_VALUE == InvertedValue.Clockwise_Positive ? -1 : 1));
    }

    private void setRightRotorPosition(double angle) {
        rightSimState.setRawRotorPosition(angle * (NewElevatorConstants.RIGHT_MOTOR_INVERTED_VALUE == InvertedValue.Clockwise_Positive ? -1 : 1));
    }

    private void setLeftRotorVelocity(double velocity) {
        leftSimState.setRotorVelocity(velocity * (NewElevatorConstants.RIGHT_MOTOR_INVERTED_VALUE == InvertedValue.Clockwise_Positive ? -1 : 1));
    }

    private void setRightRotorVelocity(double velocity) {
        rightSimState.setRotorVelocity(velocity * (NewElevatorConstants.RIGHT_MOTOR_INVERTED_VALUE == InvertedValue.Clockwise_Positive ? -1 : 1));
    }

    @Override
    public void simulationPeriodic() {
        elevatorSim.setInputVoltage((getLeftMotorVoltage() + getRightMotorVoltage()) / 2);
        elevatorSim.update(0.02);
        
        // double sprocketCircumference = Math.PI * NewElevatorConstants.SPOOL_DIAMETER.in(Meters);
        double position = elevatorSim.getPositionMeters();
        double velocity = elevatorSim.getVelocityMetersPerSecond();
        double motorAngle = position * NewElevatorConstants.SENSOR_MECHANISM_RATIO;
        double motorVelocity = velocity * NewElevatorConstants.SENSOR_MECHANISM_RATIO;

        setLeftRotorPosition(motorAngle);
        setRightRotorPosition(motorAngle);

        setLeftRotorVelocity(motorVelocity);
        setRightRotorVelocity(motorVelocity);

        leftSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        rightSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    }
}
