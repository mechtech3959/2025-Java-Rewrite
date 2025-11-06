package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public interface ElevatorIO {
        public LoggedMechanism2d e_Mech = new LoggedMechanism2d(2, 8);
        public LoggedMechanismRoot2d e_Root = e_Mech.getRoot("elevator",2,0);
        public LoggedMechanismLigament2d e_StageOne = new LoggedMechanismLigament2d("Stage 1", 8, 90);
        public LoggedMechanismLigament2d e_StageTwo = new LoggedMechanismLigament2d("Stage 2", 5, 90);
        public LoggedMechanismLigament2d e_StageThree = new LoggedMechanismLigament2d("Stage 3", 2, 90);
        
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
        public boolean isAtTarget = false;
        public LoggedMechanism2d elevator2d = e_Mech;
            }
    
    default void init(){
        e_Root.append(e_StageOne);
        e_Root.append(e_StageTwo);
        e_Root.append(e_StageThree);
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
    default void simulationInit(){}
}
