package frc.robot.subsystems.Claw;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Robot;

public class ClawMotorIO implements ClawIO {
    private TalonFX axisMotor= new TalonFX(Constants.CanIdConstants.clawAxisMotorId, Constants.CanIdConstants.canbus);
    private CANcoder axisEncoder = new CANcoder(Constants.CanIdConstants.clawAxisEncoderId, Constants.CanIdConstants.canbus);
    private ClawConfig config;
    private double lastKnownAngle = 0.0;
    MotionMagicVoltage positionVoltage = new MotionMagicVoltage(0).withSlot(0);
    @Override
    public void setAxis(double angle) {
        axisMotor.setControl(positionVoltage.withPosition(angle).withEnableFOC(true).withUseTimesync(true));
        lastKnownAngle = angle;
    }
    @Override
    public double getAxis(){
        return axisMotor.getPosition().getValueAsDouble();
    }
    @Override
    public void configure(TalonFXConfiguration talonConfig,CANcoderConfiguration cancoderConfig){
        axisMotor.getConfigurator().apply(config.AxisMotorConfig(Constants.CanIdConstants.clawAxisMotorId));
        axisEncoder.getConfigurator().apply(config.encoderConfig());

    }
    @Override
 public boolean acceptableAngle() {
    if (Robot.isReal()) {
      if ((getAxis() == lastKnownAngle) ||
          ((getAxis() >= lastKnownAngle - 0.08) &&
              (getAxis() <= lastKnownAngle + 0.08))) {
        return true;
      } else {
        return false;
      }
    } else {
      return true;
    }
  }

    @Override
    public Command moveAxis(double pose) {
    return Commands.runOnce(() -> {
      axisMotor.setControl(positionVoltage.withPosition(pose).withEnableFOC(true).withUseTimesync(true));
    });}
}

