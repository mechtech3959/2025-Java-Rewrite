package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;

import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;


public class L4 extends Command {
    public final ClawSubsystem c;
    public final ElevatorSubsystem e;
    public L4(ClawSubsystem Claw,ElevatorSubsystem Elevator){
        c = Claw;
        e = Elevator;
        addRequirements(getRequirements());
    }
    @Override
    public void initialize(){

    } 
    @Override
    public void execute(){
        
    }
    @Override
    public boolean isFinished(){
        return true;
    }
}
