package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.MetersPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.LinearVelocityUnit;
import frc.robot.Telemetry;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Claw.ClawSubsystem;
import frc.robot.subsystems.Claw.ClawSubsystem.ClawStates;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem.ElevatorStates;

public class SuperStructureSubsystem extends SubsystemBase {
    private final ElevatorSubsystem elevator;
    private final ClawSubsystem claw;
    private final CommandSwerveDrivetrain drivetrain;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
    private final Telemetry logger = new Telemetry(MaxSpeed);

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
        Processor,
        Test
    }

    private superState setSuperState = superState.Home;

    public SuperStructureSubsystem(ElevatorSubsystem elevator, ClawSubsystem claw, CommandSwerveDrivetrain drivetrain) {
        this.elevator = elevator;
        this.claw = claw;
        this.drivetrain = drivetrain;
        drivetrain = TunerConstants.createDrivetrain();

    }

    //@SuppressWarnings("static-access")
    void setState() {

        switch (setSuperState) {

            case Home:
                elevator.elevatorState = elevator.elevatorState.Home;
                elevator.changeState(elevator.elevatorState.Home);
                break;
            case L1:
                claw.clawState = claw.clawState.L1;
                if (claw.data.acceptableAngle == true)
                elevator.changeState(elevator.elevatorState.L1);
                break;
            case L2:
                claw.clawState = claw.clawState.L2;
                if (claw.data.acceptableAngle == true)
                elevator.changeState(elevator.elevatorState.L2);
                break;
            case L3:
                claw.clawState = claw.clawState.L3;
                if (claw.data.acceptableAngle == true)
                elevator.changeState(elevator.elevatorState.L3);
                break;
            case L4:
                claw.clawState = claw.clawState.L4;
                if (claw.data.acceptableAngle == true)
                elevator.changeState(elevator.elevatorState.L4);
                break;
            case Net:
                break;
            case DeAlgea_L2:
                claw.clawState = claw.clawState.Algea;

                if (claw.data.acceptableAngle == true)
                elevator.changeState(elevator.elevatorState.DeAlgea_L2);
                break;
            case DeAlgea_L3:
                claw.clawState = claw.clawState.Algea;

                if (claw.data.acceptableAngle == true)
                    elevator.changeState(elevator.elevatorState.DeAlgea_L3);
                break;
            case Processor:
                claw.clawState = claw.clawState.Algea;

                if (claw.data.acceptableAngle == true)
                elevator.changeState(ElevatorStates.L1);//check
                break;
            case Intake:
                elevator.elevatorState = elevator.elevatorState.Home;
              //  while (!claw.feedIO.hasCoral())
              //      claw.clawState = claw.clawState.Intake;
              //  if ()
              //      claw.clawState = claw.clawState.L1;
                break;
            case Test:
               // claw.clawState = ClawStates.L1;
                 claw.clawState = claw.clawState.L1;

                break;
            default:
                break;
        }
    }

    public void SubTelemetry() {
        drivetrain.registerTelemetry(logger::telemeterize);
        Logger.recordOutput("/3D/Drive/Pose", new Pose3d(drivetrain.getState().Pose));
        Logger.recordOutput("/3D/Elevator/1stStage",
                new Pose3d(0.0, elevator.visualizeElevatorOutput(), 0.0, new Rotation3d()));
        Logger.recordOutput("/3D/Elevator/Carrige",
                new Pose3d(0.0, elevator.data.encoderPosition, 0.0, new Rotation3d()));
        Logger.recordOutput("accept", claw.data.acceptableAngle);
        Logger.recordOutput("State/Super", setSuperState);
        Logger.recordOutput("State/Claw", claw.clawState);
        Logger.recordOutput("State/Elevator", elevator.elevatorState);

    }

    public void changeState(superState state) {
        setSuperState = state;
    }

    @Override
    public void periodic() {
        SubTelemetry();
        setState();
        super.periodic();
    }

}
