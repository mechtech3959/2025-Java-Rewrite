package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class ElevatorTalonFXIO implements ElevatorIO {
    private TalonFX masterM = new TalonFX(Constants.CanIdConstants.ElevatorSMotorId, Constants.CanIdConstants.canbus);
    private TalonFX slaveM = new TalonFX(Constants.CanIdConstants.ElevatorMMotorId, Constants.CanIdConstants.canbus);
    private CANcoder elevatorEncoder = new CANcoder(Constants.CanIdConstants.ElevatorEncoderId,
            Constants.CanIdConstants.canbus);
    private ElevatorConfig config = new ElevatorConfig();
    private MotionMagicVoltage elevatorMotion = new MotionMagicVoltage(0);
    private double target = 0.0;
    
    

    @Override
    public void configure() {
        masterM.getConfigurator().apply(config.ElevatorMotorConfig());
        slaveM.getConfigurator().apply(config.ElevatorMotorConfig());
        elevatorEncoder.getConfigurator().apply(config.elevatorEncoderConfig());
    }

    @Override
    public void setHeight(double pose) {
        masterM.setControl(elevatorMotion.withPosition(pose).withEnableFOC(true).withUseTimesync(true));
        target = pose;
    }

    @Override
    public double getHeight() {
        return elevatorEncoder.getPosition().getValueAsDouble();
    }

    @Override
    public boolean isAtTarget() {
        if (target == getHeight()) {
            // Further this later.. like acceptable angle with claw.
            return true;
        } else {
            return false;
        }
    }
    @Override 
    public void updateData(elevatorData data){
          data.masterMPosition = masterM.getPosition().getValueAsDouble();
          data.masterMVelocity = masterM.getVelocity().getValueAsDouble();
          data.MasterMinputVolts = masterM.getMotorVoltage().getValueAsDouble();
          data.slaveMPosition = slaveM.getPosition().getValueAsDouble();
          data.slaveMVelocity = slaveM.getPosition().getValueAsDouble();
          data.targetPose = target;
          

    }
    @Override 
    public void  periodic(){

    }
}
