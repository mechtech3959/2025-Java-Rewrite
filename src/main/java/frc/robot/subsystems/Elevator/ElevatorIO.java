package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public class ElevatorIOInputs {
        public double masterMPosition = 0;
        public double masterMVelocity = 0;
        public double slaveMPosition = 0;
        public double slaveMVelocity = 0;
        public double encoderPosition = 0;
        public double encoderVelocity = 0;
        public double targetPose = 0;
        public double MasterMinputVolts = 0;
        public double MasterMinputCurrentDraw = 0;
        public String getAppliedControl = "def";

    }

    default void resetAxis() {
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

    default void updateInputs(ElevatorIOInputs inputs) {
    }
    default void periodic(){}

}
