package frc.robot.subsystems.Vision;

import frc.robot.subsystems.Vision.LimelightHelpers;

import java.security.Timestamp;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLightSubsystem extends SubsystemBase {
  public double TX;
   public double TY;
   public double TA;
   public boolean TV;
   public String pipeLine;
   public LimelightHelpers.PoseEstimate limelightMeasurement;
   public Pose2d foundPosition;
   public double timeStamp;
   

    public LimeLightSubsystem(String llName) {
        pipeLine = llName;
    }

    public Command place() {
        return null;
    }
    public void setRobot(double yaw){
        LimelightHelpers.SetRobotOrientation(pipeLine,yaw,0,0,0,0,0);
    
    }
    public void updateTracking() {
        TX = LimelightHelpers.getTX(pipeLine);
        TY = LimelightHelpers.getTY(pipeLine);
        TA = LimelightHelpers.getTA(pipeLine);
        TV = LimelightHelpers.getTV(pipeLine);
        limelightMeasurement =  LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(pipeLine);
        if(limelightMeasurement != null){
        foundPosition = limelightMeasurement.pose;
        timeStamp = limelightMeasurement.timestampSeconds;
        }
    }

    

    @Override
    public void periodic() {
        updateTracking();
        super.periodic();
    }
}
