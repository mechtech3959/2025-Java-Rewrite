package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.MetersPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

import frc.robot.Telemetry;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem.ClawStates;
import frc.robot.subsystems.claw.ClawSubsystem.FeedStates;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorStates;

public class SuperStructureSubsystem extends SubsystemBase {
    public final ElevatorSubsystem elevator;
    public final ClawSubsystem claw;
    private final CommandSwerveDrivetrain drivetrain;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
    private final Telemetry logger = new Telemetry(MaxSpeed);
    public boolean coral;

    public enum superState {
        Tare(0),
        Home(1),
        Intake(2),
        Processor(3),
        L1(4),
        L2(5),
        DeAlgea_L2(6),
        DeAlgea_L3(7),
        L3(8),
        L4(9),
        Net(10);

        private final int id;

        private superState(int i) {
            this.id = i;
        }

        public final int getId() {
            return id;
        }
    }

    public superState setSuperState = superState.Home;
    private FeedStates setFeed = FeedStates.Off;
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
                home();
                break;
            case L1:
                l1();
                break;
            case L2:
                l2();
                break;
            case L3:
                l3();
                break;
            case L4:
                l4();
                break;
            case Net:
                net();
                break;
            case DeAlgea_L2:
                deAlgea_L2();
                break;
            case DeAlgea_L3:
                deAlgea_L3();
                break;
            case Processor:
                processor();
                break;
            case Intake:
                intake();
                break;
            case Tare:
                tareSuper();
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
    private void l1() {
        claw.changeState(ClawStates.L1);
        if (claw.isAcceptable()) {
            elevator.changeState(ElevatorStates.L1);
        } else {
            l1();
        }
    }

    private void l2() {
        claw.changeState(ClawStates.L2);
        if (claw.isAcceptable()) {
            elevator.changeState(ElevatorStates.L2);
        } else {
            l2();
        }
    }

    private void l3() {
        claw.changeState(ClawStates.L3);
        if (claw.isAcceptable()) {
            elevator.changeState(ElevatorStates.L3);
        } else {
            l3();
        }
    }

    private void l4() {
        claw.changeState(ClawStates.L4);
        if (claw.isAcceptable()) {
            elevator.changeState(ElevatorStates.L4);
        } else {
            l4();
        }
    }

    private void home() {
        claw.changeState(ClawStates.Travel);
        if (claw.isAcceptable()) {
            elevator.changeState(ElevatorStates.Home);
        } else {
            home();
        }
    }

    private void deAlgea_L2() {
        claw.changeState(ClawStates.Algea);
        if (claw.isAcceptable()) {
            elevator.changeState(ElevatorStates.DeAlgea_L2);
        } else {
            deAlgea_L2();
        }
    }

    private void deAlgea_L3() {
        claw.changeState(ClawStates.Algea);
        if (claw.isAcceptable()) {
            elevator.changeState(ElevatorStates.DeAlgea_L3);
        } else {
            deAlgea_L3();
        }
    }

    private void intake() {
        elevator.changeState(ElevatorStates.Home);
        claw.changeState(ClawStates.Travel);

        if (!claw.hasCoral()) {
            if (elevator.getHeight() <= 2.3)
                claw.changeState(ClawStates.Intake);
            coral = false;
        } else {
            coral = true;
            claw.changeState(ClawStates.Travel);
            changeState(superState.Home);
        }

    }

    private void processor() {
        claw.changeState(ClawStates.Process);
        if (claw.isAcceptable()) {
            elevator.changeState(ElevatorStates.L1);
        } else {
            processor();
        }
    }

    private void net() {

    }

    // data collection for subsystems
    public void SubTelemetry() {
        drivetrain.registerTelemetry(logger::telemeterize);
        // The 3d allows visualization of the robot
        Logger.recordOutput("3D/Drive/Pose", new Pose3d(drivetrain.getState().Pose));
        /// this is the only one outside of its respective subsystem because it's height
        /// relies on elevator pos
        Logger.recordOutput("3D/Claw", new Pose3d(0, 0, elevator.convert(), new Rotation3d(0, claw.getAxis(), 0)));
        Logger.recordOutput("State/Super", setSuperState);

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

    public superState getState() {
        return setSuperState;
    }

    private void tareSuper() {
        elevator.tareElevator();
        claw.tareClaw();
    }

    @Override
    public void periodic() {
        SubTelemetry();
        setState();
        super.periodic();
    }

}
