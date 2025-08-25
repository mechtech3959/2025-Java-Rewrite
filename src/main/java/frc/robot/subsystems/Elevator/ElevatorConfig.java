package frc.robot.subsystems.Elevator;

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
public class ElevatorConfig {
    public static final FeedbackSensorSourceValue fused = FeedbackSensorSourceValue.FusedCANcoder;

    public TalonFXConfiguration ElevatorMotorConfig(){
         MotionMagicConfigs motion = new MotionMagicConfigs().withMotionMagicCruiseVelocity(40)
                .withMotionMagicAcceleration(40)
                .withMotionMagicJerk(1600).withMotionMagicExpo_kA(0.3);
        Slot0Configs slot = new Slot0Configs().withGravityType(GravityTypeValue.Elevator_Static).withKP(7).withKI(1.2)
                .withKD(0.1).withKS(0.4).withKV(0.001).withKA(0).withKG(0.75)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        TalonFXConfiguration elevatorConfig = new TalonFXConfiguration().withFeedback(new FeedbackConfigs()
                .withFeedbackSensorSource(fused).withFeedbackRemoteSensorID(9)
                .withRotorToSensorRatio(18)
                .withSensorToMechanismRatio(1))
                .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake))
                .withMotionMagic(motion)
                .withSlot0(slot).withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs().withReverseSoftLimitThreshold(0.03).withForwardSoftLimitEnable(false))
                .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLowerLimit(20).withSupplyCurrentLimit(50).withSupplyCurrentLimitEnable(true));
     
                return elevatorConfig;
                    }
    public CANcoderConfiguration elevatorEncoderConfig(){
        CANcoderConfiguration elevatorEncConfig = new CANcoderConfiguration()
        .withMagnetSensor(new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.Clockwise_Positive).withAbsoluteSensorDiscontinuityPoint(0.5));
                return elevatorEncConfig;
    }
}
