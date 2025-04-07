package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
@SuppressWarnings("unused")
public class ElevatorSubsystem extends SubsystemBase {
TalonFX masterM;
TalonFX slaveM;
CANcoder elevatorEncoder;
MotionMagicVoltage elevatorMotion;

public ElevatorSubsystem (){
    masterM = new TalonFX(19, "CanBus");
    slaveM = new TalonFX(20,"CanBus");
    elevatorEncoder = new CANcoder(9,"CanBus");
    config();
}

public Command place(){
    return runOnce(null);
}
public void config(){
    MotionMagicConfigs motion = new MotionMagicConfigs().withMotionMagicCruiseVelocity(40)
    .withMotionMagicAcceleration(40)
    .withMotionMagicJerk(2000).withMotionMagicExpo_kA(0.3);
    Slot0Configs slot  = new Slot0Configs().withGravityType(GravityTypeValue.Elevator_Static).withKP(7).withKI(0.8).withKD(0.1).withKS(0.4).withKV(0.001).withKA(0).withKG(0.3).withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    TalonFXConfiguration elevatorConfig = 
    new TalonFXConfiguration().withFeedback(new FeedbackConfigs()
    .withFusedCANcoder(elevatorEncoder)
    .withRotorToSensorRatio(1)
    .withSensorToMechanismRatio(18))
    .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive).withNeutralMode(NeutralModeValue.Brake))
    .withMotionMagic(motion)
    .withSlot0(slot).withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLowerLimit(30).withSupplyCurrentLimit(60).withSupplyCurrentLowerTime(1));
  CANcoderConfiguration elevatorEncConfig = new CANcoderConfiguration().withMagnetSensor(new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.Clockwise_Positive).withAbsoluteSensorDiscontinuityPoint(0.5));
  masterM.getConfigurator().apply(elevatorConfig);
  slaveM.getConfigurator().apply(elevatorConfig);
  elevatorEncoder.getConfigurator().apply(elevatorEncConfig);
}
public void setHeight(){}
public void getHeight(){}
 public void coastOut(){}
 public boolean isAtTarget(){
    return true;
 }
public void sendData(){}
@Override
public void periodic() {
    // TODO Auto-generated method stub
    super.periodic();
}

}
