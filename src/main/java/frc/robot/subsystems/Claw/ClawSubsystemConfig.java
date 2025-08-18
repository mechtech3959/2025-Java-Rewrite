package frc.robot.subsystems.Claw;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

public class ClawSubsystemConfig {
    public static final FeedbackSensorSourceValue fused = FeedbackSensorSourceValue.FusedCANcoder;
    public TalonFXConfiguration AxisMotorConfig(int encoder){
  MotionMagicConfigs motion = new MotionMagicConfigs().withMotionMagicCruiseVelocity(80)
        .withMotionMagicAcceleration(80)
        .withMotionMagicJerk(1600);
    Slot0Configs slot = new Slot0Configs().withGravityType(GravityTypeValue.Arm_Cosine).withKP(12).withKI(2).withKD(0.1)
        .withKS(0.3).withKV(0.1).withKA(0).withKG(0.3)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    TalonFXConfiguration axisConfig = new TalonFXConfiguration().withFeedback(new FeedbackConfigs()
        .withFeedbackRemoteSensorID(encoder)
        .withFeedbackSensorSource(fused)
        .withRotorToSensorRatio(36)
        .withSensorToMechanismRatio(1)).withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs().withReverseSoftLimitThreshold(0.03).withReverseSoftLimitEnable(true).withForwardSoftLimitThreshold(3.14).withForwardSoftLimitEnable(true))
        .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake))
        .withMotionMagic(motion)
        .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(10).withSupplyCurrentLimitEnable(true))
        .withSlot0(slot);
    return axisConfig;
    }
    public CANcoderConfiguration encoderConfig(){
        CANcoderConfiguration axisEncConfig = new CANcoderConfiguration().withMagnetSensor(new MagnetSensorConfigs()
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive).withAbsoluteSensorDiscontinuityPoint(0.5));
    return axisEncConfig;
    }

    
    
}
