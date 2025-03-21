package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class ClawSubsystem extends SubsystemBase {
public ClawSubsystem (){
}
public Command place(){
    return runOnce(null);
}
  public void setAxis(){}
  public void getAxis(){}
  public void setIntake(){}
  public void setFeed(){}
  public void setFeedStop(){}  
  public void setStaticIntake(){}
  public void setStaticOutake(){}  
  public boolean hasCoral(){
    return true;
  }
  public boolean acceptableAngle(){
    return true;
  }
@Override
public void periodic() {
    // TODO Auto-generated method stub
    super.periodic();
}

}
