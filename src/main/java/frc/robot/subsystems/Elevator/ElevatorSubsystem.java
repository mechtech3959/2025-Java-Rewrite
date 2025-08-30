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
import org.littletonrobotics.junction.AutoLogOutput;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

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
    public ElevatorSim elevatorSim;
    public ElevatorSim carriagElevatorSim;
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

        elevatorSim = new ElevatorSim(DCMotor.getFalcon500Foc(2), 18, 1, 1, 0, 0.93, true, 0.01, 0.000, 0.000);
        carriagElevatorSim = new ElevatorSim(DCMotor.getFalcon500Foc(2), 18, 1, 1, 0, 1.8, true, 0.01,
                0.000, 0.000);
        blue = new Color8Bit(0, 0, 255);
        elevatorMech = new LoggedMechanism2d(20, 50, blue);
        root = elevatorMech.getRoot("elev", 10, 0);
        elevatorLin = root.append(new LoggedMechanismLigament2d("elev", elevatorSim.getPositionMeters(), 90));
        elevatorMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500Foc(2), 0.1, 18), DCMotor.getKrakenX60Foc(2));
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
                elevatorIO.setHeight(3.8);

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
        elevatorSim.setInputVoltage(12);
        elevatorSim.setInput(0);
        carriagElevatorSim.setInputVoltage(12);
        carriagElevatorSim.setInput(0);
    }

    public void simSetHeight(double rot) {
    }

    @Override
    public void simulationPeriodic() {
        // TalonFXSimState simMaster = masterM.getSimState();
        // TalonFXSimState simSlave = slaveM.getSimState();
        // CANcoderSimState simEncoder = elevatorEncoder.getSimState();
        // simMaster.setSupplyVoltage(12);
        elevatorMotorSim.update(0.02);
        elevatorSim.setState(elevatorMotorSim.getAngularPositionRotations(), 1);
        carriagElevatorSim.setInput(target);
        elevatorSim.update(0.02);
        carriagElevatorSim.update(0.02);
    }

    // Elevator is a 2 stage cascade so the 1st stage can only physically reach 0.93
    // meters;
    // this just makes sure the robot is properly visualized during replay or live
    // visualization

    public double visualizeElevatorOutput() {
        if (data.encoderPosition > 0.93) {
            return 0.93;
        } else {
            return data.encoderPosition;
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

    @Override
    public void periodic() {
        elevatorIO.updateInputs(data);
        Logger.processInputs("Elevator", data);
        Logger.recordOutput("State/Elevator", elevatorState);
        setStates();
        sendData();
        SmartDashboard.putString("applied", data.getAppliedControl);
        super.periodic();
    }

}
