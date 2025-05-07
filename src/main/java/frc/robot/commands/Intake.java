package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;


public class Intake extends Command {
    public final ClawSubsystem c;
    public final ElevatorSubsystem e;
    public Intake(ClawSubsystem Claw,ElevatorSubsystem Elevator){
        c = Claw;
        e = Elevator;
        addRequirements(getRequirements());
    }
    @Override
    public void initialize(){
        e.setHeight(0.0);

    } 
    @Override
    public void execute(){
        if(e.isAtTarget()){
            c.setAxis(0.00);
            do {
                c.setIntake();
            } while (!c.hasCoral());
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
