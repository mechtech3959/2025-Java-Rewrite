package frc.robot.subsystems.Claw;

import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.hal.simulation.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

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

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.BooleanSupplier;

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
public  double lastKnownAngle;
     



   public enum clawState {
    Intake,
    Travel,
    L1,
    L2,
    L3,
    L4,
    algea;
  };

  clawState state;

  public ClawSubsystem() {
    axisMotor = new TalonFX(14, "CanBus");
    axisEncoder = new CANcoder(15, "CanBus");
    feedMotor = new SparkMax(30, MotorType.kBrushless);
    axisMotion = new MotionMagicVoltage(0);
    zeroAxis = new MotionMagicVoltage(0);
    lastKnownAngle = 0;
    config();
  }

  public void config() {
    MotionMagicConfigs motion = new MotionMagicConfigs().withMotionMagicCruiseVelocity(80)
        .withMotionMagicAcceleration(80)
        .withMotionMagicJerk(1600);
    Slot0Configs slot = new Slot0Configs().withGravityType(GravityTypeValue.Arm_Cosine).withKP(12).withKI(2).withKD(0.1)
        .withKS(0.3).withKV(0.1).withKA(0).withKG(0.3)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    TalonFXConfiguration axisConfig = new TalonFXConfiguration().withFeedback(new FeedbackConfigs()
        .withFusedCANcoder(axisEncoder)
        .withRotorToSensorRatio(36)
        .withSensorToMechanismRatio(1))
        .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake))
        .withMotionMagic(motion)
        .withSlot0(slot);
    CANcoderConfiguration axisEncConfig = new CANcoderConfiguration().withMagnetSensor(new MagnetSensorConfigs()
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive).withAbsoluteSensorDiscontinuityPoint(0.5));
    axisMotor.getConfigurator().apply(axisConfig);
    axisEncoder.getConfigurator().apply(axisEncConfig);
  }

  public Command place() {
    return runOnce(null);
  }

  public void setAxis(Double pose) {
    axisMotor.setControl(axisMotion.withPosition(pose).withEnableFOC(true));
    if(RobotBase.isReal()){lastKnownAngle = axisMotor.getPosition().getValueAsDouble();
    }else{

      lastKnownAngle = pose;

    }}

  public double getAxis() {
    return axisMotor.getPosition().getValueAsDouble();
  }

  public void setIntake() {
    if (hasCoral()) {
      feedMotor.set(0);
    } else {
      feedMotor.set(-0.2);
    }
  }

  public void setFeed(double output) {
    feedMotor.set(output);
  }

  public void setFeedStop() {
    feedMotor.set(0);
  }

  public void setStaticIntake() {
  }

  public void setStaticOutake() {
  }

  public boolean hasCoral() {
    if (feedMotor.getAnalog().getVoltage() >= 2.9) {
      return true;
    } else {
      return false;
    }
  }

  public boolean acceptableAngle() {
    if ((getAxis() == lastKnownAngle) ||
        ((getAxis() >= lastKnownAngle - 5) &&
            (getAxis() <= lastKnownAngle + 5))) {
      return true;
    } else {
      return false;
    }
  }

  public SingleJointedArmSim sim = new SingleJointedArmSim(DCMotor.getFalcon500Foc(1), 36,
      SingleJointedArmSim.estimateMOI(0.1, 12), 0.1, 0, 270, true, 0, 0.0,0.0);
  public LoggedMechanism2d clawsim = new LoggedMechanism2d(0.1, 0.1, new Color8Bit(0,0,255));
    LoggedMechanismRoot2d root = clawsim.getRoot("cl", 0.3, 0.4);
 public LoggedMechanismLigament2d lig = root.append(new LoggedMechanismLigament2d("cl", 0.1, 90));
  public LoggedMechanismLigament2d flat =  root.append(new LoggedMechanismLigament2d("flat", 0.2, 0));
  public void simulationInit() {
    sim.setInputVoltage(12);
    sim.getInput(0);
  }

  public void simulationPeriodic() {
    double rad = ((lastKnownAngle * (180/Math.PI)) - 90) * (Math.PI/180);
    sim.update(0.05);
    sim.setState(rad, 1);
    lig.setAngle(lastKnownAngle);
SmartDashboard.putData("cc" , clawsim);
  }

  @Override
  public void periodic() {
    hasCoral();
    getAxis();
    acceptableAngle();
    super.periodic();
  }

}
