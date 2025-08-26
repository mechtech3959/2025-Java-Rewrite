package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    class elevatorData {
        public double masterMPosition;
        public double masterMVelocity;
        public double slaveMPosition;
        public double slaveMVelocity;
        public double encoderPosition;
        public double encoderVelocity;
        public double targetPose;
        public double MasterMinputVolts;
        public double MasterMinputCurrentDraw;
        public String getAppliedControl;

    }
    default void resetAxis(){}
    default void setHeight(double pose) {
    }

    default double getHeight() {
        return 0.0;
    }

    default boolean isAtTarget() {
        return false;
    }

    default void configure() {
    }

    default void updateData(elevatorData data) {
    }
}
