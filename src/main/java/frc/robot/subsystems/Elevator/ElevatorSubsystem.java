package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOInputsAutoLogged;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;

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
    double simPose = 0;

    Color8Bit blue;
    public LoggedMechanism2d elevatorMech;
    LoggedMechanismRoot2d root;
    LoggedMechanismLigament2d elevatorLin;
    DCMotorSim elevatorMotorSim;
    TalonFXSimState simMaster;
    TalonFXSimState simSlave;
    CANcoderSimState simEncoder;
    private final ElevatorIO elevatorIO;
    private final ElevatorIOInputsAutoLogged data = new ElevatorIOInputsAutoLogged();

    public ElevatorStates elevatorState = ElevatorStates.Home;

    public ElevatorSubsystem(ElevatorIO elevatorIO) {
        elevatorIO.configure();
        this.elevatorIO = elevatorIO;

        if (Robot.isSimulation()) {
            // simMaster = masterM.getSimState();
            // simSlave = slaveM.getSimState();
            // simEncoder = elevatorEncoder.getSimState();
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

    public void simSetHeight(double rot) {
    }

    @Override
    public void simulationPeriodic() {
        // TalonFXSimState simMaster = masterM.getSimState();
        // TalonFXSimState simSlave = slaveM.getSimState();
        // CANcoderSimState simEncoder = elevatorEncoder.getSimState();
        // simMaster.setSupplyVoltage(12);
        // elevatorMotorSim.update(0.02);
        // elevatorSim.setState(elevatorMotorSim.getAngularPositionRotations(), 1);
        // carriagElevatorSim.setInput(target);
        // elevatorSim.update(0.02);
        // carriagElevatorSim.update(0.02);
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
     //   changeState(ElevatorStates.Home);
        elevatorIO.resetAxis();

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
        Logger.recordOutput("Elevator/ist", isTarget());
        Logger.recordOutput("Elevator/istint", elevatorIO.isAtTarget());
 

        setStates();
        isTarget();
        sendData();
        SmartDashboard.putString("applied", data.getAppliedControl);
        super.periodic();
    }

}
