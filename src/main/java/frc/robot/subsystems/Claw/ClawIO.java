package frc.robot.subsystems.Claw;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public interface ClawIO {

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

}