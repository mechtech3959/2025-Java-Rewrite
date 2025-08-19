package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Claw.ClawSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;

public class scoreL1 extends SequentialCommandGroup {

    public final ElevatorSubsystem elevator;
    public final ClawSubsystem claw;

    public scoreL1(ElevatorSubsystem _ElevatorSubsystem, ClawSubsystem _ClawSubsystem) {
        elevator = _ElevatorSubsystem;
        claw = _ClawSubsystem;
        /* 
        addCommands(
                Commands.runOnce(() -> claw.setAxis(0.349)),
                Commands.runOnce(() -> elevator.setHeight(1.0)).onlyIf(() -> claw.acceptableAngle()),
                Commands.runOnce(() -> claw.setFeed(-0.2)),
                Commands.waitUntil(() -> !claw.hasCoral()).andThen(Commands.waitSeconds(1)),
                Commands.parallel(Commands.runOnce(() -> claw.setFeed(0.0)),
                        Commands.runOnce(() -> elevator.setHeight(0.0))),
                Commands.runOnce(() -> claw.setAxis(0.0)).onlyIf(() -> elevator.isAtTarget()));
 */   }

}
