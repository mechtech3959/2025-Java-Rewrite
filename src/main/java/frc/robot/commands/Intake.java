package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SuperStructureSubsystem;
import frc.robot.subsystems.SuperStructureSubsystem.superState;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem.ClawStates;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class Intake extends SequentialCommandGroup {

    public final SuperStructureSubsystem superstruct;

    public Intake(SuperStructureSubsystem _SuperStructureSubsystem ){
        this.superstruct = _SuperStructureSubsystem;
    addCommands(
        Commands.runOnce(()-> _SuperStructureSubsystem.changeState(superState.Intake)),
        Commands.waitUntil(()->_SuperStructureSubsystem.setSuperState == superState.Home));
    
    }

}
