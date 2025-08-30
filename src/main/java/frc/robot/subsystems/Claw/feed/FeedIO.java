package frc.robot.subsystems.Claw.feed;

import org.littletonrobotics.junction.AutoLog;

public interface FeedIO {
    @AutoLog
  class FeedIOInputs {
      public boolean hasCoral;

    }
    default void Stop(){}
    default void setIntake(double percentOut) {
    }

    default void stdIntake() {
    }

    default void intakeProcess() {
    }

    default boolean hasCoral() {
        return false;
    }
    default void updateInputs(FeedIOInputs data){}

}
