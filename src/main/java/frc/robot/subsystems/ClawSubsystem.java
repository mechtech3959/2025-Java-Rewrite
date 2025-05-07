package frc.robot.subsystems;
 import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
import com.ctre.phoenix6.configs.ExternalFeedbackConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

@SuppressWarnings("unused")
public class ClawSubsystem extends SubsystemBase {
  TalonFX axisMotor;
  CANcoder axisEncoder;
  SparkMax feedMotor;
  MotionMagicVoltage axisMotion;
  MotionMagicVoltage zeroAxis;
  double lastKnownAngle;
 public enum clawState{
    Intake,
    Travel,
    L1,
    L4
  };
  clawState state;

public ClawSubsystem (){
  axisMotor = new TalonFX(14, "CanBus");
  axisEncoder = new CANcoder(15, "CanBus");
  feedMotor = new SparkMax(30, MotorType.kBrushless);
  axisMotion = new MotionMagicVoltage(0);
  zeroAxis = new MotionMagicVoltage(0);
  lastKnownAngle = 0;
  config();
}
public void config(){
  MotionMagicConfigs motion = new MotionMagicConfigs().withMotionMagicCruiseVelocity(80)
  .withMotionMagicAcceleration(80)
  .withMotionMagicJerk(1600);
  Slot0Configs slot  = new Slot0Configs().withGravityType(GravityTypeValue.Arm_Cosine).withKP(12).withKI(2).withKD(0.1).withKS(0.3).withKV(0.1).withKA(0).withKG(0.3).withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
  TalonFXConfiguration axisConfig = 
  new TalonFXConfiguration().withFeedback(new FeedbackConfigs()
  .withFusedCANcoder(axisEncoder)
  .withRotorToSensorRatio(36)
  .withSensorToMechanismRatio(1))
  .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive).withNeutralMode(NeutralModeValue.Brake))
  .withMotionMagic(motion)
  .withSlot0(slot);
CANcoderConfiguration axisEncConfig = new CANcoderConfiguration().withMagnetSensor(new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.Clockwise_Positive).withAbsoluteSensorDiscontinuityPoint(0.5));
axisMotor.getConfigurator().apply(axisConfig);
axisEncoder.getConfigurator().apply(axisEncConfig);
}

public Command place(){
    return runOnce(null);
}
  public void setAxis(Double pose){
    axisMotor.setControl(axisMotion.withPosition(pose).withEnableFOC(true));
    lastKnownAngle = axisMotor.getPosition().getValueAsDouble();
  }
  public double getAxis(){
    return axisMotor.getPosition().getValueAsDouble();
  }
  public void setIntake(){
    if(hasCoral()){
      feedMotor.set(0);
    }else{
      feedMotor.set(-0.2);
    }
  }
  public void setFeed(double output){
    feedMotor.set(output);
  }
  public void setFeedStop(){
    feedMotor.set(0);
  }  
  public void setStaticIntake(){}
  public void setStaticOutake(){}  
  
  public boolean hasCoral(){
    if(feedMotor.getAnalog().getVoltage() >= 2.9 ){
       return true;}
       else{
        return false;}
  }
  public boolean acceptableAngle(){
    if ((getAxis() == lastKnownAngle) ||
    ((getAxis() >= lastKnownAngle - 5) &&
     (getAxis() <= lastKnownAngle + 5))) {
  return true;
} else {
  return false;
}  }
    
@Override
public void periodic() {
  hasCoral();
  getAxis();
    // TODO Auto-generated method stub
    super.periodic();
}

}
