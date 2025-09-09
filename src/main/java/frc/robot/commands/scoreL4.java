package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SuperStructureSubsystem;
import frc.robot.subsystems.SuperStructureSubsystem.superState;
import frc.robot.subsystems.claw.ClawSubsystem.FeedStates;

@SuppressWarnings("unused")
public class scoreL4 extends SequentialCommandGroup {
    private final SuperStructureSubsystem superStruct;

    public scoreL4(SuperStructureSubsystem _SuperStructureSubsystem) {
        superStruct = _SuperStructureSubsystem;
        addCommands(
                Commands.runOnce(() -> _SuperStructureSubsystem.changeState(superState.L4)),
                Commands.waitUntil(_SuperStructureSubsystem.elevator.isTarget()),
                Commands.runOnce(() -> _SuperStructureSubsystem.changeState(FeedStates.Outake)),
                // this might do some weird stuff....
                Commands.waitUntil(_SuperStructureSubsystem.claw.supplyCoral()),
                Commands.runOnce(() -> _SuperStructureSubsystem.changeState(superState.Home, FeedStates.Off)));

    }

}