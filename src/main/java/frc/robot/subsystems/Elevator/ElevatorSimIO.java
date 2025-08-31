package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismObject2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.math.system.plant.LinearSystemId;

import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;

public class ElevatorSimIO implements ElevatorIO {
    private TalonFX masterM = new TalonFX(Constants.CanIdConstants.ElevatorSMotorId, Constants.CanIdConstants.canbus);
    private TalonFX slaveM = new TalonFX(Constants.CanIdConstants.ElevatorMMotorId, Constants.CanIdConstants.canbus);
    private CANcoder elevatorEncoder = new CANcoder(Constants.CanIdConstants.ElevatorEncoderId,
            Constants.CanIdConstants.canbus);
    private ElevatorConfig config = new ElevatorConfig();
    private TalonFXSimState masterSimM;
    private TalonFXSimState slaveSimM;
    private CANcoderSimState elevatorEncoderSim;
     double target = 0;
    double simPose = 0;
    public ElevatorSim elevatorSim = new ElevatorSim(DCMotor.getFalcon500Foc(2), 18, 1, 1, 0, 0.93, true, 0.01, 0.000, 0.000);
    ;
    public ElevatorSim  carriagElevatorSim = new ElevatorSim(DCMotor.getFalcon500Foc(2), 18, 1, 1, 0, 1.8, true, 0.01,
    0.000, 0.000);
    Color8Bit blue = new Color8Bit(0, 0, 255);
    public LoggedMechanism2d elevatorMech = new LoggedMechanism2d(20, 50, blue);
    LoggedMechanismRoot2d root = elevatorMech.getRoot("elev", 10, 0);
    LoggedMechanismLigament2d  elevatorLin = root.append(new LoggedMechanismLigament2d("elev", elevatorSim.getPositionMeters(), 90));
    DCMotorSim  elevatorMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500Foc(2), 0.1, 18), DCMotor.getKrakenX60Foc(2));

    @Override
    public void configure() {
        masterM.getConfigurator().apply(config.ElevatorMotorConfig());
        slaveM.getConfigurator().apply(config.ElevatorMotorConfig());
        elevatorEncoder.getConfigurator().apply(config.elevatorEncoderConfig());
        resetAxis();
        // slave follows master to ensure motors arent fighting each other when
        // following motion profiles
        slaveM.setControl(new StrictFollower(masterM.getDeviceID()));
        masterSimM = masterM.getSimState();
        slaveSimM = slaveM.getSimState();
        elevatorEncoderSim = elevatorEncoder.getSimState();
    }
}
