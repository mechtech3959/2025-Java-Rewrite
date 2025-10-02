package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SuperStructureSubsystem;
import frc.robot.subsystems.SuperStructureSubsystem.superState;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem.ClawStates;
import frc.robot.subsystems.claw.ClawSubsystem.FeedStates;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorStates;

public class scoreL1 extends SequentialCommandGroup {


        private final SuperStructureSubsystem superStruct;

        public scoreL1(SuperStructureSubsystem _SuperStructureSubsystem) {
            superStruct = _SuperStructureSubsystem;
            addCommands(
                    Commands.runOnce(() -> _SuperStructureSubsystem.changeState(superState.L1)),
                    Commands.waitUntil(_SuperStructureSubsystem.elevator.isTarget()),
                    Commands.runOnce(() -> _SuperStructureSubsystem.changeState(FeedStates.Outake)),
                    // this might do some weird stuff....
                    Commands.waitUntil(_SuperStructureSubsystem.claw.supplyCoral()),
                    Commands.runOnce(() -> _SuperStructureSubsystem.changeState(superState.Home, FeedStates.Off)));
    
        }
    
    }