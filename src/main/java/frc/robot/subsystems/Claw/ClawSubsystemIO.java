package frc.robot.subsystems.Claw;

public interface ClawSubsystemIO {

    default void setAxis(double angle){}    
    default void getAxis(){}    
    default void resetAxis(){}
    

} 