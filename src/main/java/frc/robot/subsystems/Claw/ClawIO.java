package frc.robot.subsystems.Claw;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public interface ClawIO {

    default void setAxis(double angle) {
    }

    default void getAxis() {
    }

    default void resetAxis() {
    }

    default void configure(TalonFXConfiguration talonConfig,CANcoderConfiguration cancoderConfig) {
    }

}