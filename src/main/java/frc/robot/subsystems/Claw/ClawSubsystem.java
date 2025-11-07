package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.claw.feed.FeedIO;
import frc.robot.subsystems.claw.feed.FeedIOInputsAutoLogged;

import frc.robot.subsystems.claw.ClawIO;
import frc.robot.subsystems.claw.ClawIO.ClawIOInputsAutoLogged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ClawSubsystem extends SubsystemBase {

  private final ClawIO clawIO;
  private final FeedIO feedIO;
  private final ClawIOInputsAutoLogged dataClaw = new ClawIOInputsAutoLogged();
  private final FeedIOInputsAutoLogged dataFeed = new FeedIOInputsAutoLogged();

  public enum ClawStates {
    L1,
    L2,
    L3,
    L4,
    Travel,
    Home,
    Algea,
    Process,
    Intake,
    Zero
  }

  public enum FeedStates {
    Off,
    Outake,
    Intake,
    PercentOut,
    Algea
  }

  public SingleJointedArmSim sim;
  public LoggedMechanism2d clawMech;
  public LoggedMechanismLigament2d pivot;
  public LoggedMechanismLigament2d flat;

  public ClawStates clawState = ClawStates.Home;
  public FeedStates feedState = FeedStates.Off;
  public double percentOut = 0.0;

  public ClawSubsystem(ClawIO clawIO, FeedIO feedIO) {
    clawIO.configure();
    this.clawIO = clawIO;
    this.feedIO = feedIO;

    sim = new SingleJointedArmSim(DCMotor.getFalcon500Foc(1), 36,
        SingleJointedArmSim.estimateMOI(0.1, 12), 0.1, 0, 4.71, true, 0, 0.0, 0.0);
    clawMech = new LoggedMechanism2d(0.1, 0.1, new Color8Bit(0, 0, 255));
    LoggedMechanismRoot2d root = clawMech.getRoot("root", 0.02, 0.04);
    pivot = root.append(new LoggedMechanismLigament2d("pivot", 0.01, 90));
    flat = root.append(new LoggedMechanismLigament2d("flat", 0.02, 0));

    if (Robot.isSimulation()) {
      simulationInit();
    }
  }

  // Finite State Machine
  public void setStates() {
    switch (clawState) {
      case L1:
        clawIO.setAxis(0.349066);// 20 deg 0.349066
        break;
      case L2:
        clawIO.setAxis(0.349066);
        break;
      case L3:
        clawIO.setAxis(0.349066);
        break;
      case L4:
        clawIO.setAxis(0.52);

        break;
      case Algea:
        clawIO.setAxis(2.617);// 150 deg

        break;
      case Process:
        clawIO.setAxis(2.0944);
        break;
      case Home:
        clawIO.setAxis(0.0);

        break;
      case Travel:
        clawIO.setAxis(0.174);// 10 deg

        break;
      case Intake:
        clawIO.setAxis(0);
        changeState(FeedStates.Intake);
        break;
      case Zero:
        clawIO.resetAxis();
        break;
      default:
        break;
    }

    switch (feedState) {
      case Intake:
        if (!feedIO.hasCoral()) {
          feedIO.stdIntake();
        } else {
          feedIO.Stop();
        }

        break;
      case Outake:
        feedIO.stdIntake();
        break;
      case PercentOut:
        feedIO.setIntake(percentOut);
        break;
      case Algea:
        // check what is needed for static intake of algea
        feedIO.setIntake(-0.1);
        break;

      case Off:
        feedIO.Stop();
        break;
      default:
        break;
    }
  }

  public void simulationInit() {
    sim.setInputVoltage(12);
    sim.setInput(0, 0); // Set simulation input to a default value

  }

  public double getAxis() {
    return clawIO.getAxis();
  }

  public boolean isAcceptable() {
    return clawIO.acceptableAngle();
  }

  public BooleanSupplier supplyAcceptable() {
    return () -> clawIO.acceptableAngle();
  }

  public boolean hasCoral() {
    if (Robot.isReal()) {
      return feedIO.hasCoral();
    } else {
      return true;
    }
  }

  // should send the opposite as has coral...
  public BooleanSupplier noCoral() {
    if (Robot.isReal()) {
      return () -> !feedIO.hasCoral();
    } else {
      return () -> true;
    }
  }

  @Override
  public void simulationPeriodic() {
    sim.update(0.05);
    // sim.setState(lastKnownAngle, 1);
    // pivot.setAngle(lastKnownAngle);
    SmartDashboard.putData("cc", clawMech);
  }

  public void sendData() {
    // Logger.recordOutput("Real/Claw/Axis", lastKnownAngle);
    // Logger.recordOutput("Real/Claw/Axis Speed",
    // axisEncoder.getVelocity().getValueAsDouble());
    // Logger.recordOutput("Real/Claw/Indexer Speed", feedMotor.get());
  }

  // Function to change the state of claw outside of this subsystem
  public void changeState(ClawStates state) {
    clawState = state;

  }

  public void changeState(FeedStates feed) {
    feedState = feed;
  }

  public void changeState(ClawStates state, FeedStates feed) {
    clawState = state;
    feedState = feed;

  }

  public void changeState(ClawStates state, FeedStates feed, double percent) {
    clawState = state;
    feedState = feed;
    percentOut = percent;

  }

  public void changeState(FeedStates feed, double percent) {
    feedState = feed;
    percentOut = percent;

  }

  public void tareClaw() {
    changeState(ClawStates.Home, FeedStates.Off);
    clawIO.resetAxis();

  }

  @Override
  public void periodic() {
    clawIO.updateInputs(dataClaw);
    feedIO.updateInputs(dataFeed);
    Logger.processInputs("Subsystem/Claw/", dataClaw);
    Logger.processInputs("Subsystem/Feed/", dataFeed);
    Logger.recordOutput("State/Claw", clawState);

    clawIO.periodic();
    getAxis();
    isAcceptable();
    hasCoral();
    supplyAcceptable();
    noCoral();
    setStates();
    sendData();

    super.periodic();
  }

}
