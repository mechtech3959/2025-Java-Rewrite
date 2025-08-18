package frc.robot.subsystems.Claw;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Velocity;

public class ClawSubsystemMotorIO implements ClawSubsystemIO {
    private TalonFX axisMotor;
    MotionMagicVoltage positionVoltage = new MotionMagicVoltage(0).withSlot(0);

    @Override
    public void setAxis(double angle) {
        axisMotor.setControl(positionVoltage.withPosition(angle).withEnableFOC(true).withUseTimesync(true));
    }
    @Override
    public void configure(TalonFXConfiguration talonConfig,CANcoderConfiguration cancoderConfig){
        axisMotor.getConfigurator().apply(talonConfig);

    }
}
