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
    Commands.runOnce(() -> claw.setAxis(30.0)),
    Commands.runOnce(() -> elevator.setHeight(1.0)).onlyIf(() -> claw.acceptableAngle()),
    Commands.runOnce(() -> claw.setFeed(-0.2)), 
    Commands.waitUntil(() -> !claw.hasCoral()),
    Commands.parallel( Commands.runOnce(() -> claw.setFeed(0.0)), Commands.runOnce(() -> elevator.setHeight(0.0))),
    Commands.runOnce(()-> claw.setAxis(0.0)).onlyIf(()-> elevator.isAtTarget())
   ); 
}

}
