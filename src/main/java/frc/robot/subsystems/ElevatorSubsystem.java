package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class ElevatorSubsystem extends SubsystemBase {
public ElevatorSubsystem (){
}
public Command place(){
    return runOnce(null);
}
 public void setHeight(){}
public void getHeight(){}
 public void coastOut(){}
 public boolean isAtTarget(){
    return true;
 }
public void sendData(){}
@Override
public void periodic() {
    // TODO Auto-generated method stub
    super.periodic();
}

}
