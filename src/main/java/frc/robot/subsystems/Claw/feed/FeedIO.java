package frc.robot.subsystems.Claw.feed;

public interface FeedIO {

    default void setIntake(double percentOut) {
    }

    default void stdIntake() {
    }

    default void intakeProcess() {
    }

    default boolean hasCoral() {
        return false;
    }

}
