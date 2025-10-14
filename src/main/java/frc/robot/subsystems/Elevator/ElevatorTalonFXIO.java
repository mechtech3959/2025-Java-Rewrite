package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class ElevatorTalonFXIO implements ElevatorIO {
    private TalonFX masterM = new TalonFX(Constants.CanIdConstants.ElevatorSMotorId, Constants.CanIdConstants.canbus);
    private TalonFX slaveM = new TalonFX(Constants.CanIdConstants.ElevatorMMotorId, Constants.CanIdConstants.canbus);
    private CANcoder elevatorEncoder = new CANcoder(Constants.CanIdConstants.ElevatorEncoderId,
            Constants.CanIdConstants.canbus);
    private ElevatorConfig config = new ElevatorConfig();
    private MotionMagicVoltage elevatorMotion = new MotionMagicVoltage(0.0).withSlot(0);
    private double target = 0.0;

    // makes sure configs are applied during startup and are not retained
    // force set positions to 0 to fix offsets(mostly only matters for testing)
    @Override
    public void configure() {
        masterM.getConfigurator().apply(config.ElevatorMotorConfig());
        slaveM.getConfigurator().apply(config.ElevatorMotorConfig());
        elevatorEncoder.getConfigurator().apply(config.elevatorEncoderConfig());
        resetAxis();
        // slave follows master to ensure motors arent fighting each other when
        // following motion profiles
        slaveM.setControl(new StrictFollower(masterM.getDeviceID()));
    }

    @Override
    public void resetAxis() {
        masterM.setPosition(0.0);
        slaveM.setPosition(0.0);
        elevatorEncoder.setPosition(0.0);
    }

    // set position only if target has changed
    // if changed motion profile moves elevator to set position
    @Override
    public void setHeight(double pose) {
         if(pose != target){
        masterM.setControl(elevatorMotion.withPosition(pose).withEnableFOC(true).withUseTimesync(true)  );
         target = pose;}
    }

    @Override
    public double getHeight() {
        return elevatorEncoder.getPosition().getValueAsDouble();
    }

    @Override
    public boolean isAtTarget() {
        if (target == getHeight() || (target <= getHeight()-0.05) && (target >= getHeight() +0.05)) {
            // Further this later.. like acceptable angle with claw.
            return true;
        } else {
            return false;
        }
    }

    // this makes sure log outputs are up to date(when called periodically)
    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.masterMPosition = masterM.getPosition().getValueAsDouble();
        inputs.masterMVelocity = masterM.getVelocity().getValueAsDouble();
        inputs.MasterMinputVolts = masterM.getMotorVoltage().getValueAsDouble();
        inputs.slaveMPosition = slaveM.getPosition().getValueAsDouble();
        inputs.slaveMVelocity = slaveM.getPosition().getValueAsDouble();
        inputs.targetPose = target;
        inputs.getAppliedControl = masterM.getAppliedControl().toString();

    }

}
