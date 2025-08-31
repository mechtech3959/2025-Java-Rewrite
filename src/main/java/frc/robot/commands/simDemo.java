package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

@SuppressWarnings("unused")
public class simDemo extends SequentialCommandGroup {

    public final ElevatorSubsystem elevator;
    public final ClawSubsystem claw;

    public simDemo(ElevatorSubsystem _ElevatorSubsystem, ClawSubsystem _ClawSubsystem) {
        elevator = _ElevatorSubsystem;
        claw = _ClawSubsystem;
        /*addCommands(
                Commands.runOnce(() -> claw.setAxis(0.349)),
                Commands.runOnce(() -> elevator.setHeight(1)),
                Commands.waitSeconds(1.0),
                Commands.runOnce(() -> claw.setAxis(0.349)),
                Commands.runOnce(() -> elevator.setHeight(2.3)),
                Commands.waitSeconds(1.0), 
                Commands.runOnce(() -> claw.setAxis(0.349)),
                Commands.runOnce(() -> elevator.setHeight(4.2)),
                Commands.waitSeconds(1.0),
                Commands.runOnce(() -> claw.setAxis(2.61)),
                Commands.runOnce(() -> elevator.setHeight(5.2)),
                Commands.waitSeconds(1.0),
                Commands.runOnce(() -> claw.setAxis(0.698)),
                Commands.runOnce(() -> elevator.setHeight(4)),
                Commands.waitSeconds(1.0), 
                Commands.runOnce(() -> claw.setAxis(0.349)),
                Commands.runOnce(() -> elevator.setHeight(3)),
                Commands.waitSeconds(1.0),
                Commands.runOnce(() -> claw.setAxis(0.349)),
                Commands.runOnce(() -> elevator.setHeight(2)),
                Commands.waitSeconds(1.0),
                Commands.runOnce(() -> elevator.setHeight(0.0)),
                Commands.runOnce(() -> claw.setAxis(0.0)).onlyIf(() -> elevator.isAtTarget()));
   */ }

}