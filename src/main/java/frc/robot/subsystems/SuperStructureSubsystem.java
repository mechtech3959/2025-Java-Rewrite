package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.L2;
import frc.robot.commands.L3;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Claw.ClawSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;

public class SuperStructureSubsystem extends SubsystemBase {
    private final ElevatorSubsystem elevator;
    private final ClawSubsystem claw;
    private final CommandSwerveDrivetrain drivetrain;
    enum superState{
        Home,
        L1,
        L2,
        L3,
        L4,
        Net
    }
    public SuperStructureSubsystem(ElevatorSubsystem elevator, ClawSubsystem claw, CommandSwerveDrivetrain drivetrain)
    {   this.elevator = elevator;
        this.claw = claw;
        this.drivetrain = drivetrain;
        drivetrain = TunerConstants.createDrivetrain();    



    }
    @Override
    public void periodic(){

    }
    
}
