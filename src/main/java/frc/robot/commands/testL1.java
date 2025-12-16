package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

@SuppressWarnings("unused")
public class testL1 extends SequentialCommandGroup {

  // public final ElevatorSubsystem elevator;
  // public final ClawSubsystem claw;

  public testL1(ElevatorSubsystem elevator, ClawSubsystem claw) {
    // elevator = _ElevatorSubsystem;
    // claw = _ClawSubsystem;
    /*
     * addCommands(
     * Commands.runOnce(() -> {claw.setAxis(0.174);}),
     * Commands.waitUntil(() ->
     * claw.acceptableAngle()).andThen(()->elevator.setHeight(1.0)),
     * Commands.waitSeconds(2.00),
     * Commands.runOnce(() -> elevator.setHeight(0.0)).onlyIf(() ->
     * claw.acceptableAngle()));
     */ }

}
