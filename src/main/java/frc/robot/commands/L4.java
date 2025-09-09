package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem.ClawStates;
import frc.robot.subsystems.claw.ClawSubsystem.FeedStates;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorStates;

@SuppressWarnings("unused")
public class L4 extends SequentialCommandGroup {

  public final ElevatorSubsystem elevator;
  public final ClawSubsystem claw;

  public L4(ElevatorSubsystem _ElevatorSubsystem, ClawSubsystem _ClawSubsystem) {
    elevator = _ElevatorSubsystem;
    claw = _ClawSubsystem;
    addCommands(

        Commands.runOnce(() -> _ClawSubsystem.changeState(ClawStates.L4))
            .andThen(() -> _ElevatorSubsystem.changeState(ElevatorStates.L4)));
    Commands.waitUntil(_ElevatorSubsystem.isTarget());
    Commands.runOnce(() -> _ClawSubsystem.changeState(FeedStates.Outake));
    Commands.waitUntil(_ClawSubsystem.supplyCoral());
    Commands.runOnce(() -> _ElevatorSubsystem.changeState(ElevatorStates.Home));
    Commands.runOnce(() -> _ClawSubsystem.changeState(ClawStates.Travel, FeedStates.Off));

  }

}
