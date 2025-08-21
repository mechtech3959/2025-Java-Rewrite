package frc.robot.subsystems.Claw.feed;

public interface FeedIO {
    public class feedData{
        boolean hasCoral;

    }
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
