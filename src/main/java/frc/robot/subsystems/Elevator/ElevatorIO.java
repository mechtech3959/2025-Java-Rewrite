package frc.robot.subsystems.Elevator;

public interface ElevatorIO {
    default void setHeight(double pose){}
    default double getHeight(){return 0.0;}
    default boolean isAtTarget(){return false;}
    
    default void configure(){}
    
}
