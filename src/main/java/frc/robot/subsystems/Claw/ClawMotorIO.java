package frc.robot.subsystems.Claw;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Velocity;
import frc.robot.Constants;

public class ClawMotorIO implements ClawIO {
    private TalonFX axisMotor= new TalonFX(Constants.CanIdConstants.clawAxisMotorId, Constants.CanIdConstants.canbus);
    private CANcoder axisEncoder = new CANcoder(Constants.CanIdConstants.clawAxisEncoderId, Constants.CanIdConstants.canbus);
    private ClawConfig config;
    MotionMagicVoltage positionVoltage = new MotionMagicVoltage(0).withSlot(0);

    @Override
    public void setAxis(double angle) {
        axisMotor.setControl(positionVoltage.withPosition(angle).withEnableFOC(true).withUseTimesync(true));
    }
    @Override
    public void configure(TalonFXConfiguration talonConfig,CANcoderConfiguration cancoderConfig){
        axisMotor.getConfigurator().apply(config.AxisMotorConfig(Constants.CanIdConstants.clawAxisMotorId));
        axisEncoder.getConfigurator().apply(config.encoderConfig());

    }
}
