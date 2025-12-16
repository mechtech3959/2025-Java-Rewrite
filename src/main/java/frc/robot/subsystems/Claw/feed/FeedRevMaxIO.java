package frc.robot.subsystems.claw.feed;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;

public class FeedRevMaxIO implements FeedIO {
    SparkMax feedMotor = new SparkMax(Constants.CanIdConstants.clawIntakeMotorId, MotorType.kBrushless);
    @Override
    public void Stop(){
        feedMotor.set(0);
    }
    @Override
    public void setIntake(double percentOut) {
        feedMotor.set(percentOut);
    }

    @Override
    public void stdIntake() {
        feedMotor.set(-0.2);
    }
    // the sensor indicates detection by changing voltage of analog
    @Override
    public boolean hasCoral() {
        if (feedMotor.getAnalog().getVoltage() >= 2.9) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void intakeProcess() {
        if (hasCoral() == false) {
            stdIntake();
        } else {
            setIntake(0);
        }
    }
    @Override
    public void updateInputs(FeedIOInputs data){
        data.hasCoral = hasCoral();
    }
}
