package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.MetersPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

import frc.robot.Telemetry;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Claw.ClawIO;
import frc.robot.subsystems.Claw.ClawSubsystem;
import frc.robot.subsystems.Claw.ClawSubsystem.ClawStates;
import frc.robot.subsystems.Claw.ClawSubsystem.FeedStates;
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
        Test,
        Test2,
        Test3
    }

    private superState setSuperState = superState.Home;
    private FeedStates setFeed = FeedStates.Home;
    private double clawSetOut = 0.0;
    public SuperStructureSubsystem(ElevatorSubsystem elevator, ClawSubsystem claw, CommandSwerveDrivetrain drivetrain) {
        // external extries of subsystems are equal to local declarations
        this.elevator = elevator;
        this.claw = claw;
        this.drivetrain = drivetrain;
        drivetrain = TunerConstants.createDrivetrain();

    }

    void setState() {

        switch (setSuperState) {

            case Home:
                Home();
                break;
            case L1:
                L1();
                break;
            case L2:
                L2();
                break;
            case L3:
                L3();
                break;
            case L4:
                L4();
                break;
            case Net:
                Net();
                break;
            case DeAlgea_L2:
                DeAlgea_L2();
                break;
            case DeAlgea_L3:
                DeAlgea_L3();
                break;
            case Processor:
                Processor();
                break;
            case Intake:
                Intake();
                break;
            case Test:
                // claw.clawState = ClawStates.L1;
                elevator.changeState(ElevatorStates.L3);
                break;
            case Test2:
                // claw.clawState = ClawStates.L1;
                elevator.changeState(ElevatorStates.L4);
                break;
            case Test3:
                // claw.clawState = ClawStates.L1;
                elevator.changeState(ElevatorStates.Home);
                break;
            default:
                break;
        }
        switch (setFeed) {
            case Intake:
                claw.changeState(setFeed);
                break;
            case Outake:
            claw.changeState(setFeed);
            break;
            case PercentOut:
            claw.changeState(setFeed, clawSetOut);
            break;
            case Algea:
            claw.changeState(setFeed);
            break;
            default:
                break;
        }
    }

    // Almost all of these are recursive statements(meaning they call themselves in
    // the definitions)
    // this is done because the function is only called once and if the claw is not
    // where it is supposed to be
    // the elevator will not move
    private void L1() {
        claw.changeState(ClawStates.L1);
        if (claw.data.acceptableAngle) {
            elevator.changeState(ElevatorStates.L1);
        } else {
            L1();
        }
    }

    private void L2() {
        claw.changeState(ClawStates.L2);
        if (claw.data.acceptableAngle) {
            elevator.changeState(ElevatorStates.L2);
        } else {
            L2();
        }
    }

    private void L3() {
        claw.changeState(ClawStates.L3);
        if (claw.data.acceptableAngle) {
            elevator.changeState(ElevatorStates.L3);
        } else {
            L3();
        }
    }

    private void L4() {
        claw.changeState(ClawStates.L4);
        if (claw.data.acceptableAngle) {
            elevator.changeState(ElevatorStates.L4);
        } else {
            L4();
        }
    }

    private void Home() {
        claw.changeState(ClawStates.Travel);
        if (claw.data.acceptableAngle) {
            elevator.changeState(ElevatorStates.Home);
        } else {
            Home();
        }
    }

    private void DeAlgea_L2() {
        claw.changeState(ClawStates.Algea);
        if (claw.data.acceptableAngle) {
            elevator.changeState(ElevatorStates.DeAlgea_L2);
        } else {
            DeAlgea_L2();
        }
    }

    private void DeAlgea_L3() {
        claw.changeState(ClawStates.Algea);
        if (claw.data.acceptableAngle) {
            elevator.changeState(ElevatorStates.DeAlgea_L3);
        } else {
            DeAlgea_L3();
        }
    }

    private void Intake() {
        elevator.changeState(ElevatorStates.Home);
        if (!claw.dataF.hasCoral) {
            claw.changeState(ClawStates.Intake);
        } else {
            claw.changeState(ClawStates.Travel);
        }
    }

    private void Processor() {
        claw.changeState(ClawStates.Algea);
        if (claw.data.acceptableAngle) {
            elevator.changeState(ElevatorStates.L1);
        } else {
            Processor();
        }
    }
    
    private void Net() {

    }

    // data collection for subsystems
    public void SubTelemetry() {
        drivetrain.registerTelemetry(logger::telemeterize);
        // The 3d allows visualization of the robot
        Logger.recordOutput("3D/Drive/Pose", new Pose3d(drivetrain.getState().Pose));
        Logger.recordOutput("3D/Elevator/1stStage",
                new Pose3d(0.0,0.0, elevator.visualizeElevatorOutput(), new Rotation3d(0,0,0)));
        Logger.recordOutput("3D/Elevator/Carrige",
                new Pose3d(0.0,0.0, elevator.data.encoderPosition, new Rotation3d(0,0,0)));
        // keep outputs logged for diagnosis
        Logger.recordOutput("Claw/accept", claw.data.acceptableAngle);
        Logger.recordOutput("Claw/Target pose", claw.data.targetPose);
        Logger.recordOutput("Claw/EncPose", Units.radiansToDegrees(claw.data.clawAxis));
        Logger.recordOutput("State/Super", setSuperState);
        Logger.recordOutput("State/Claw", claw.clawState);
        Logger.recordOutput("State/Elevator", elevator.elevatorState);

    }

    // Function for external alterations of the state
    public void changeState(superState state) {
        setSuperState = state;

    }
    public void changeState(superState state, FeedStates feed) {
        setSuperState = state;
        setFeed = feed;

    }
    public void changeState(superState state, FeedStates feed, double percent) {
        setSuperState = state;
        setFeed = feed;
        clawSetOut = percent;
    }
    public void changeState(FeedStates feed, double percent) {
        setFeed = feed;
        clawSetOut = percent;
    }  
     public void changeState(FeedStates feed) {
        setFeed = feed;
    }
// fix ABOVE REDUNDANT
    @Override
    public void periodic() {
        SubTelemetry();
        setState();
        super.periodic();
    }

}
