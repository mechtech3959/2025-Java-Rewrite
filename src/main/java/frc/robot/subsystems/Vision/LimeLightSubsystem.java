package frc.robot.subsystems.Vision;

import frc.robot.subsystems.Vision.LimelightHelpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLightSubsystem extends SubsystemBase {
  public double TX;
   public double TY;
   public double TA;
   public boolean TV;
   public String pipeLine;
   public LimelightHelpers.PoseEstimate limelightMeasurement;
    public LimeLightSubsystem(String llName) {
        pipeLine = llName;
    }

    public Command place() {
        return null;
    }

    public void updateTracking() {
        TX = LimelightHelpers.getTX(pipeLine);
        TY = LimelightHelpers.getTY(pipeLine);
        TA = LimelightHelpers.getTA(pipeLine);
        TV = LimelightHelpers.getTV(pipeLine);
        limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(pipeLine);
    
    }

    public Pose2d poseEst() {
        return null;
    }

    @Override
    public void periodic() {
        updateTracking();
        super.periodic();
    }
}
