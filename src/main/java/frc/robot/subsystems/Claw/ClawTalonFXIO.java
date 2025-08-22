package frc.robot.subsystems.Claw;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Robot;

public class ClawTalonFXIO implements ClawIO {
  private TalonFX axisMotor = new TalonFX(Constants.CanIdConstants.clawAxisMotorId, Constants.CanIdConstants.canbus);
  private CANcoder axisEncoder = new CANcoder(Constants.CanIdConstants.clawAxisEncoderId,
      Constants.CanIdConstants.canbus);
  private ClawConfig config = new ClawConfig();
  private double lastKnownAngle = 0.0;
  private MotionMagicVoltage positionVoltage = new MotionMagicVoltage(0.0).withSlot(0);

  // uses motion profile to go to set position
  @Override
  public void setAxis(double angle) {
    if (angle != lastKnownAngle) {
      axisMotor.setControl(
          positionVoltage.withPosition(Units.radiansToRotations(angle)).withEnableFOC(true).withUseTimesync(true));
      lastKnownAngle = angle;
    }
  }

  @Override
  public double getAxis() {
    return axisMotor.getPosition().getValueAsDouble();
  }

  // Make sure configs are applied on startup and not retained.
  // Make sure poses are set to zero on startus
  @Override
  public void configure() {
    axisMotor.getConfigurator().apply(config.AxisMotorConfig(Constants.CanIdConstants.clawAxisMotorId));
    axisEncoder.getConfigurator().apply(config.encoderConfig());
    axisMotor.setPosition(0.0);
    axisEncoder.setPosition(0.0);

  }

  // if claw is in tolerance of setpoint. this is used to prevent collison with
  // elevator
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

  // Command test, same as setAxis but in command format
  @Override
  public Command moveAxis(double pose) {
    return Commands.runOnce(() -> {
      axisMotor.setControl(positionVoltage.withPosition(pose).withEnableFOC(true).withUseTimesync(true));
    });
  }

  // this makes sure log outputs are up to date(when called periodically)
  @Override
  public void updateInput(clawData data) {
    data.acceptableAngle = acceptableAngle();
    data.clawAxis = axisEncoder.getPosition().getValueAsDouble();
    data.clawMotorPose = axisMotor.getPosition().getValueAsDouble();
    data.clawMotorVelocity = axisMotor.getVelocity().getValueAsDouble();
    data.currentDraw = axisMotor.getBridgeOutput().getValueAsDouble();// i have no idea what this is
  }
}
