package frc.robot.subsystems.Claw;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public interface ClawIO {
    @AutoLog
    class ClawIOInputs {
        public double clawAxis;
        public double clawMotorPose;
        public double clawMotorVelocity;
        public boolean acceptableAngle;
        public double currentDraw;
        public double targetPose;

    }

    default void setAxis(double angle) {
    }

    default double getAxis() {
        return 0.0;
    }

    default void resetAxis() {
    }

    default void configure() {
    }

    default Command moveAxis(double pose) {
        return Commands.none();
    }

    default boolean acceptableAngle() {
        return true;
    }
    default void updateInputs(ClawIOInputs data){
    
    }

}