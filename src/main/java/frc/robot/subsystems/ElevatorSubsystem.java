package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import java.util.logging.Logger;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import edu.wpi.first.hal.simulation.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;

@SuppressWarnings("unused")
public class ElevatorSubsystem extends SubsystemBase {
    TalonFX masterM;
    TalonFX slaveM;
    CANcoder elevatorEncoder;
    MotionMagicVoltage elevatorMotion;
    double target = 0;

    public ElevatorSubsystem() {
        masterM = new TalonFX(19, "CanBus");
        slaveM = new TalonFX(20, "CanBus");
        elevatorEncoder = new CANcoder(9, "CanBus");
        config();
        TalonFXSimState simMaster = masterM.getSimState();
        TalonFXSimState simSlave = slaveM.getSimState();
        CANcoderSimState simEncoder = elevatorEncoder.getSimState();

    }

    public Command place() {
        return runOnce(null);
    }

    public void config() {
        MotionMagicConfigs motion = new MotionMagicConfigs().withMotionMagicCruiseVelocity(40)
                .withMotionMagicAcceleration(40)
                .withMotionMagicJerk(2000).withMotionMagicExpo_kA(0.3);
        Slot0Configs slot = new Slot0Configs().withGravityType(GravityTypeValue.Elevator_Static).withKP(7).withKI(0.8)
                .withKD(0.1).withKS(0.4).withKV(0.001).withKA(0).withKG(0.3)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        TalonFXConfiguration elevatorConfig = new TalonFXConfiguration().withFeedback(new FeedbackConfigs()
                .withFusedCANcoder(elevatorEncoder)
                .withRotorToSensorRatio(1)
                .withSensorToMechanismRatio(18))
                .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake))
                .withMotionMagic(motion)
                .withSlot0(slot)
                .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLowerLimit(30).withSupplyCurrentLimit(60)
                        .withSupplyCurrentLowerTime(1));
        CANcoderConfiguration elevatorEncConfig = new CANcoderConfiguration()
                .withMagnetSensor(new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                        .withAbsoluteSensorDiscontinuityPoint(0.5));
        masterM.getConfigurator().apply(elevatorConfig);
        slaveM.getConfigurator().apply(elevatorConfig);
        elevatorEncoder.getConfigurator().apply(elevatorEncConfig);
    }

    public void setHeight(double pose) {
        if (RobotBase.isReal()) {
            masterM.setControl(elevatorMotion.withPosition(pose).withEnableFOC(true).withUseTimesync(true));
            target = elevatorMotion.Position;
        } else {
            target = pose;
        }
    }

    public double getHeight() {
        masterM.getPosition().getValueAsDouble();
        slaveM.getPosition().getValueAsDouble();
        return elevatorEncoder.getPosition().getValueAsDouble();
    }

    public void coastOut() {

    }

    public boolean isAtTarget() {
        if (masterM.getPosition().getValueAsDouble() == target) {
            return true;
        } else {
            return false;
        }
    }

    public ElevatorSim elevatorSim = new ElevatorSim(DCMotor.getFalcon500Foc(2), 18, 30, 1, 0, 1, true, 0.0, 0.0, 0.0);
    public ElevatorSim carriagElevatorSim = new ElevatorSim(DCMotor.getFalcon500Foc(2), 18, 30, 1, 0, 1.9, true, 0.0,
            0.0, 0.0);
    Color8Bit blue = new Color8Bit(0, 0, 255);
    public LoggedMechanism2d visElevator = new LoggedMechanism2d(20, 50, blue);
    LoggedMechanismRoot2d root = visElevator.getRoot("elev", 10, 0);
    LoggedMechanismLigament2d lig = root
            .append(new LoggedMechanismLigament2d("elev", elevatorSim.getPositionMeters(), 90));

    public void simulationInit() {
        elevatorSim.setInputVoltage(12);
        elevatorSim.setInput(0);
        carriagElevatorSim.setInputVoltage(12);
        carriagElevatorSim.setInput(0);
    }

    public void simSetHeight(double rot) {
    }

    public void simulationPeriodic() {
        lig.setLength(target);
        elevatorSim.setState(target, 1);
        carriagElevatorSim.setState(target, 1);
        elevatorSim.update(0.05);
        carriagElevatorSim.update(0.05);
        SmartDashboard.putNumber("Elevator", elevatorSim.getPositionMeters());

        SmartDashboard.putData("2dev", visElevator);

    }

    void sendData() {
    }

    @Override
    public void periodic() {
        isAtTarget();

        super.periodic();
    }

}
