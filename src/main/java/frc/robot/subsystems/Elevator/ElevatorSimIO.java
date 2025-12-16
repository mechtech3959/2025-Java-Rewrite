package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
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
    private MotionMagicVoltage elevatorMotion = new MotionMagicVoltage(0.0).withSlot(0);

    private TalonFXSimState masterSimM;
    private TalonFXSimState slaveSimM;
    private CANcoderSimState elevatorEncoderSim;
    public double target = 0;

    double simPose = 0;
    public ElevatorSim elevatorSim = new ElevatorSim(DCMotor.getFalcon500Foc(2), 18, 1, 2, 0, 0.93, true, 0.01, 0.000,
            0.000);
    public ElevatorSim carriagElevatorSim = new ElevatorSim(DCMotor.getFalcon500Foc(2), 18, 1, 1, 0, 1.8, true, 0.01,
            0.000, 0.000);
    Color8Bit blue = new Color8Bit(0, 0, 255);
    public LoggedMechanism2d elevatorMech = new LoggedMechanism2d(20, 50, blue);
    LoggedMechanismRoot2d root = elevatorMech.getRoot("elev", 10, 0);
    LoggedMechanismLigament2d elevatorLin = root
            .append(new LoggedMechanismLigament2d("elev", elevatorSim.getPositionMeters(), 90));
    DCMotorSim elevatorMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500Foc(2), 0.1, 18), DCMotor.getFalcon500Foc(2));

    @Override
    public void configure() {
        masterM.getConfigurator().apply(config.SimElevatorMotorConfig());
        slaveM.getConfigurator().apply(config.SimElevatorMotorConfig());
        elevatorEncoder.getConfigurator().apply(config.simElevatorEncoderConfig());
        resetAxis();
        // slave follows master to ensure motors arent fighting each other when
        // following motion profiles
        slaveM.setControl(new StrictFollower(masterM.getDeviceID()));
        masterSimM = masterM.getSimState();
        slaveSimM = slaveM.getSimState();
        elevatorEncoderSim = elevatorEncoder.getSimState();

    }

    @Override
    public void setHeight(double pose) {
        if (pose != target) {
            masterM.setControl(elevatorMotion.withPosition(pose).withUseTimesync(true));
            target = pose;
        }
    }

    @Override
    public double getHeight() {
        return masterM.getPosition().getValueAsDouble();
    }

    @Override
    public void resetAxis() {
        masterM.setPosition(0);
        slaveM.setPosition(0);
        elevatorMotorSim.setState(0, 0);
        // elevatorSim.setState(0, 0);
        // carriagElevatorSim.setState(0 ,0);
    }

    @Override
    public void simulationInit() {

    }

    @Override
    public boolean isAtTarget() {
        return ((target == getHeight()) || ((target >= getHeight() - 0.01) && (target <= getHeight() + 0.01))) ? true
                : false;

    }

    @Override
    public void periodic() {
        masterSimM = masterM.getSimState();
        slaveSimM = slaveM.getSimState();
        elevatorEncoderSim = elevatorEncoder.getSimState();

        masterSimM.setSupplyVoltage(RobotController.getBatteryVoltage());
        slaveSimM.setSupplyVoltage(RobotController.getBatteryVoltage());
        elevatorEncoderSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        var mMotorVolts = masterSimM.getMotorVoltageMeasure();
        // var sMotorVolts = slaveSimM.getMotorVoltageMeasure();
        elevatorMotorSim.setInputVoltage(mMotorVolts.in(Volts));
        elevatorMotorSim.update(0.02);
        masterSimM.setRawRotorPosition(elevatorMotorSim.getAngularPosition());
        masterSimM.setRotorVelocity(elevatorMotorSim.getAngularVelocity());
        slaveSimM.setRawRotorPosition(elevatorMotorSim.getAngularPosition());
        slaveSimM.setRotorVelocity(elevatorMotorSim.getAngularVelocity());
        elevatorEncoderSim.setRawPosition(elevatorMotorSim.getAngularPosition());
        elevatorEncoderSim.setVelocity(elevatorMotorSim.getAngularVelocity());
        elevatorSim.update(0.02);
        carriagElevatorSim.update(0.02);
        elevatorSim.setInputVoltage(elevatorMotorSim.getInputVoltage());
        carriagElevatorSim.setInputVoltage(elevatorMotorSim.getInputVoltage());
        elevatorSim.setInput(elevatorMotorSim.getAngularVelocityRPM() * elevatorMotorSim.getInputVoltage());
        carriagElevatorSim.setInput(elevatorMotorSim.getAngularVelocityRPM() * elevatorMotorSim.getInputVoltage());
        elevatorSim.setState(elevatorMotorSim.getAngularPositionRotations(), elevatorMotorSim.getAngularVelocityRPM());

    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.masterMPosition = masterM.getPosition().getValueAsDouble();
        inputs.masterMVelocity = masterM.getPosition().getValueAsDouble();
        inputs.MasterMinputVolts = masterM.getMotorVoltage().getValueAsDouble();
        inputs.slaveMPosition = slaveM.getPosition().getValueAsDouble();
        inputs.slaveMVelocity = slaveM.getPosition().getValueAsDouble();
        inputs.targetPose = target;
        inputs.getAppliedControl = masterM.getAppliedControl().toString();

    }
}
