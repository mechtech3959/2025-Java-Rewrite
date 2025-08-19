package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Claw.ClawSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;

public class SuperStructureSubsystem extends SubsystemBase {
    private final ElevatorSubsystem elevator;
    private final ClawSubsystem claw;
    private final CommandSwerveDrivetrain drivetrain;

    public enum superState {
        Home,
        L1,
        L2,
        L3,
        L4,
        Net,
        Intake,
        DeAlgea_L2,
        DeAlgea_L3,
        Processor
    }

    private final superState setSuperState;

    public SuperStructureSubsystem(ElevatorSubsystem elevator, ClawSubsystem claw, CommandSwerveDrivetrain drivetrain,
            superState setSuperState) {
        this.elevator = elevator;
        this.claw = claw;
        this.drivetrain = drivetrain;
        this.setSuperState = setSuperState;
        drivetrain = TunerConstants.createDrivetrain();

    }

    @SuppressWarnings("static-access")
    void setState() {

        switch (setSuperState) {

            case Home:
                elevator.elevatorState = elevator.elevatorState.Home;
                claw.clawState = claw.clawState.Home;
                break;
            case L1:
                claw.clawState = claw.clawState.L1;
                if (claw.clawIO.acceptableAngle() == true)
                    elevator.elevatorState = elevator.elevatorState.L1;
                break;
            case L2:
                claw.clawState = claw.clawState.L2;
                if (claw.clawIO.acceptableAngle() == true)
                    elevator.elevatorState = elevator.elevatorState.L2;
                break;
            case L3:
                claw.clawState = claw.clawState.L3;
                if (claw.clawIO.acceptableAngle() == true)
                    elevator.elevatorState = elevator.elevatorState.L3;
                break;
            case L4:
                claw.clawState = claw.clawState.L4;
                if (claw.clawIO.acceptableAngle() == true)
                    elevator.elevatorState = elevator.elevatorState.L4;
                break;
            case Net:
                break;
            case DeAlgea_L2:
                claw.clawState = claw.clawState.Algea;

                if (claw.clawIO.acceptableAngle() == true)
                    elevator.elevatorState = elevator.elevatorState.DeAlgea_L2;
                break;
            case DeAlgea_L3:
                claw.clawState = claw.clawState.Algea;

                if (claw.clawIO.acceptableAngle() == true)
                    elevator.elevatorState = elevator.elevatorState.DeAlgea_L3;
                break;
            case Processor:
                claw.clawState = claw.clawState.Algea;

                if (claw.clawIO.acceptableAngle() == true)
                    elevator.elevatorState = elevator.elevatorState.L1;// check
                break;
            case Intake:
                elevator.elevatorState = elevator.elevatorState.Home;
                while (!claw.feedIO.hasCoral())
                    claw.clawState = claw.clawState.Intake;
                if (claw.feedIO.hasCoral())
                    claw.clawState = claw.clawState.L1;
                break;
            default:
                break;
        }
    }

    @Override
    public void periodic() {

        setState();
    }

}
