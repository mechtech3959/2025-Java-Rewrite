package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
//import frc.robot.subsystems.elevator.ElevatorIOInputsAutoLogged;

import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class ElevatorSubsystem extends SubsystemBase {
    public enum ElevatorStates {
        L1,
        L2,
        L3,
        L4,
        Net,
        Travel,
        Home,
        DeAlgea_L2,
        DeAlgea_L3
    }

    double target = 0;

    private final ElevatorIO elevatorIO;
    private final ElevatorIOInputsAutoLogged data = new ElevatorIOInputsAutoLogged();

    public ElevatorStates elevatorState = ElevatorStates.Home;

    public ElevatorSubsystem(ElevatorIO elevatorIO) {
        elevatorIO.configure();
        this.elevatorIO = elevatorIO;

        if (Robot.isSimulation()) {

            simulationInit();
        }

    }

    // Elevators Finite State Machine
    public void setStates() {
        switch (elevatorState) {
            case Home:
                elevatorIO.setHeight(0.0);
                break;
            case L1:
                elevatorIO.setHeight(0);// check this
                break;
            case L2:
                elevatorIO.setHeight(1);

                break;
            case L3:
                elevatorIO.setHeight(2.3);

                break;
            case L4:
                elevatorIO.setHeight(4.35);

                break;
            case DeAlgea_L2:
                elevatorIO.setHeight(1.9);

                break;
            case DeAlgea_L3:
                // was 3.8
                elevatorIO.setHeight(3.4);

                break;
            case Net:
                elevatorIO.setHeight(5.2);

                break;
            case Travel:
                break;
            default:
                break;
        }

    }

    public void simulationInit() {
        elevatorIO.simulationInit();
    }

    // Elevator is a 2 stage cascade so the 1st stage can only physically reach 0.93
    // meters;
    // this just makes sure the robot is properly visualized during replay or live
    // visualization
    public double convert() {
        return data.masterMPosition * 0.3048;
    }

    public double visualizeElevatorOutput() {
        if (convert() > 0.93) {
            return 0.93;
        } else {
            return convert();
        }
    }

    // Function to alter the State Machine outside of the subsystem
    public void changeState(ElevatorStates state) {
        elevatorState = state;
    }

    void sendData() {
        // Logger.recordOutput("Real/Elevator/Position", getHeight());
        // Logger.recordOutput("Real/Elevator/Acceleration",
        // elevatorEncoder.getVelocity().getValueAsDouble());
    }

    public BooleanSupplier isTarget() {
        return () -> elevatorIO.isAtTarget();
    }

    public void tareElevator() {
        elevatorIO.resetAxis();
    }

    public double getHeight() {
        return elevatorIO.getHeight();
    }

    @Override
    public void periodic() {
        elevatorIO.periodic();
        elevatorIO.updateInputs(data);
        Logger.processInputs("Elevator", data);
        Logger.recordOutput("State/Elevator", elevatorState);
        Logger.recordOutput("3D/Elevator/1st stage",
                new Pose3d(0, 0, convert(), new Rotation3d(0, 0, 0)));
        Logger.recordOutput("3D/Elevator/carriage",
                new Pose3d(0, 0, convert(), new Rotation3d(0, 0, 0)));
        Logger.recordOutput("3D/Elevator/Stationary", new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)));
        Logger.recordOutput("Elevator/istint", elevatorIO.isAtTarget());

        setStates();
        isTarget();
        sendData();
        super.periodic();
    }

}
