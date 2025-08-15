package frc.robot.subsystems.Claw;

import com.ctre.phoenix6.hardware.TalonFX;

public class ClawSubsystemMotorIO implements ClawSubsystemIO {
    private TalonFX axisMotor;
    @Override
    public void setAxis(double angle) {
        ClawSubsystemIO.super.setAxis(angle);
    }
    
}
