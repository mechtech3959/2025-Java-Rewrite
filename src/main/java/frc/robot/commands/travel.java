package frc.robot.commands;
import frc.robot.subsystems.ClawSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class travel extends Command {
    public final ClawSubsystem c;
    public travel(ClawSubsystem claw){
        c = claw;
    }
    @Override
    public void initialize() {
        c.setAxis(20.0);
    }
    @Override
    public boolean isFinished(){
        return true;
    }
}
