package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ClawSubsystem;

public class scoreL1 extends SequentialCommandGroup {

 
    public final ElevatorSubsystem elevator;
    public final ClawSubsystem claw;

public scoreL1(ElevatorSubsystem e, ClawSubsystem c){
    elevator = e;
    claw = c;
    addCommands(
    Commands.runOnce(() -> claw.setAxis(20.0)),
    Commands.runOnce(() -> elevator.setHeight(1.0)).onlyIf(claw.accept),
    Commands.runOnce(() -> claw.setFeed(-0.2)), 
    Commands.waitUntil(claw.coral),
    Commands.parallel( Commands.runOnce(() -> claw.setFeed(0.0)), Commands.runOnce(() -> elevator.setHeight(0.0)))
   ); 
}

}
