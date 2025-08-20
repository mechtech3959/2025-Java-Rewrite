package frc.robot.subsystems.Elevator;

public interface ElevatorIO {
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

    }

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
