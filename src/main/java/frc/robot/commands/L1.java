package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Claw.ClawSubsystem;


public class L1 extends Command {
    public final ClawSubsystem c;
    public final ElevatorSubsystem e;
    public L1(ClawSubsystem Claw,ElevatorSubsystem Elevator){
        c = Claw;
        e = Elevator;
        addRequirements(getRequirements());
    }
    @Override
    public void initialize(){
        c.setAxis(20.0);
    } 
    @Override
    public void execute(){
        if(c.acceptableAngle() && c.hasCoral()){
            e.setHeight(1.0);
        }
    }
    @Override
    public boolean isFinished(){
        if(e.isAtTarget()){
            c.setAxis(30.00);
        return true;}else{
            return false; 
        }
    }
}
