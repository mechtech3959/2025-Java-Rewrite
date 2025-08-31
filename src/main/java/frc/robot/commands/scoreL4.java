package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

@SuppressWarnings("unused")
public class scoreL4 extends SequentialCommandGroup {

    public final ElevatorSubsystem elevator;
    public final ClawSubsystem claw;

    public scoreL4(ElevatorSubsystem _ElevatorSubsystem, ClawSubsystem _ClawSubsystem) {
        elevator = _ElevatorSubsystem;
        claw = _ClawSubsystem;
      /*   addCommands(
            
                Commands.runOnce(() -> claw.setAxis(0.689)),
                Commands.runOnce(() -> elevator.setHeight(4.5)).onlyIf(() -> claw.acceptableAngle()),
                Commands.runOnce(() -> claw.setFeed(-0.2)),
                Commands.waitUntil(() -> !claw.hasCoral()).andThen(Commands.waitSeconds(1)),
                Commands.parallel(Commands.runOnce(() -> claw.setFeed(0.0)),
                        Commands.runOnce(() -> elevator.setHeight(0.0))),
                Commands.runOnce(() -> claw.setAxis(0.0)).onlyIf(() -> elevator.isAtTarget()));
  */  }

}

