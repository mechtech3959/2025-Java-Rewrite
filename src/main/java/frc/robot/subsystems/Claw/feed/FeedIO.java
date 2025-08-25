package frc.robot.subsystems.Claw.feed;

import org.littletonrobotics.junction.AutoLog;

public interface FeedIO {
    @AutoLog
  class feedData {
      public boolean hasCoral;

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
    default void updateInput(feedData data){}

}
