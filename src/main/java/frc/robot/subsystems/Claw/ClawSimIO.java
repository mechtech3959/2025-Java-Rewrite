package frc.robot.subsystems.claw;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Robot;

public class ClawSimIO implements ClawIO {
      private TalonFX axisMotor = new TalonFX(Constants.CanIdConstants.clawAxisMotorId, Constants.CanIdConstants.canbus);
  private CANcoder axisEncoder = new CANcoder(Constants.CanIdConstants.clawAxisEncoderId,
      Constants.CanIdConstants.canbus);


      private ClawConfig config = new ClawConfig();
  private double lastKnownAngle = 0.0;
  private MotionMagicVoltage positionVoltage = new MotionMagicVoltage(0.0).withSlot(0);

  
  public TalonFXSimState axisSimMotor = axisMotor.getSimState();
 public CANcoderSimState axisSimEncoder = axisEncoder.getSimState();
 
 public DCMotorSim clawMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500Foc(1),0.001, 18), DCMotor.getFalcon500Foc(1));
 public SingleJointedArmSim clawSim = new SingleJointedArmSim(DCMotor.getFalcon500Foc(1), 36,
  SingleJointedArmSim.estimateMOI(0.1, 12), 0.1, 0, 4.71, true, 0, 0.0, 0.0);
public LoggedMechanism2d clawMech = new LoggedMechanism2d(0.1, 0.1, new Color8Bit(0, 0, 255));
LoggedMechanismRoot2d root = clawMech.getRoot("root", 0.02, 0.04);
public LoggedMechanismLigament2d pivot = root.append(new LoggedMechanismLigament2d("pivot", 0.01, 90));
public LoggedMechanismLigament2d flat = root.append(new LoggedMechanismLigament2d("flat", 0.02, 0));

  // uses motion profile to go to set position
  @Override
  public void setAxis(double angle) {
    if (angle != lastKnownAngle) {
      axisMotor.setControl(
          positionVoltage.withPosition(Units.radiansToRotations(angle)).withEnableFOC(false).withUseTimesync(true));
      lastKnownAngle = Units.radiansToRotations(angle);
    }
  }

  @Override
  public double getAxis() {
    return axisMotor.getPosition().getValueAsDouble();
  }

  // Make sure configs are applied on startup and not retained.
  // Make sure poses are set to zero on startus
  @Override
  public void configure() {
    axisMotor.getConfigurator().apply(config.AxisMotorConfig(Constants.CanIdConstants.clawAxisMotorId));
    axisEncoder.getConfigurator().apply(config.encoderConfig());
   
    resetAxis();

  }

  // if claw is in tolerance of setpoint. this is used to prevent collison with
  // elevator
  @Override
  public boolean acceptableAngle() {
    if (Robot.isReal()) {
      if ((getAxis() == lastKnownAngle) ||
          ((getAxis() >= lastKnownAngle - 0.08) &&
              (getAxis() <= lastKnownAngle + 0.08))) {
        return true;
      } else {
        return false;
      }
    } else {
      return true;
    }
  }
  @Override
  public void resetAxis(){
    axisMotor.setPosition(0.0);
    axisEncoder.setPosition(0.0);
  }
  // Command test, same as setAxis but in command format
  @Override
  public Command moveAxis(double pose) {
    return Commands.runOnce(() -> {
      axisMotor.setControl(positionVoltage.withPosition(pose).withEnableFOC(false).withUseTimesync(true));
    });
  }
@Override 
 public void periodic(){
  axisSimMotor = axisMotor.getSimState();
  axisSimEncoder = axisEncoder.getSimState();
  axisSimMotor.setSupplyVoltage(RobotController.getBatteryVoltage());
  axisSimEncoder.setSupplyVoltage(RobotController.getBatteryVoltage());
  var axisvolt = axisSimMotor.getMotorVoltageMeasure();
  clawMotorSim.setInputVoltage(axisSimMotor.getMotorVoltageMeasure().in(Volts));
  clawMotorSim.update(0.02);
  
  axisSimMotor.setRawRotorPosition(clawMotorSim.getAngularPosition());
  axisSimMotor.setRotorVelocity(clawMotorSim.getAngularVelocity());
  axisSimEncoder.setRawPosition(clawMotorSim.getAngularPosition());
  axisSimEncoder.setVelocity(clawMotorSim.getAngularVelocity());

  clawSim.setInputVoltage(clawMotorSim.getInputVoltage());
  clawSim.update(0.02);

  clawSim.setInput(clawMotorSim.getAngularVelocityRPM() * clawMotorSim.getInputVoltage());
  clawSim.setState(clawMotorSim.getAngularPositionRotations(), clawMotorSim.getAngularVelocityRPM());
 }
  // this makes sure log outputs are up to date(when called periodically)
  @Override
  public void updateInputs(ClawIOInputs data) {
    data.acceptableAngle = acceptableAngle();
    data.clawAxis =  clawMotorSim.getAngularPositionRad(); // axisEncoder.getPosition().getValueAsDouble();
    data.clawMotorPose = axisMotor.getPosition().getValueAsDouble();
    data.clawMotorVelocity = axisMotor.getVelocity().getValueAsDouble();
    data.currentDraw = axisMotor.getBridgeOutput().getValueAsDouble();// i have no idea what this is
    data.targetPose = lastKnownAngle;
  }
}
