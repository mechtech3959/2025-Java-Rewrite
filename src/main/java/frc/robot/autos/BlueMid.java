package frc.robot.autos;

import java.nio.channels.Pipe;

import choreo.Choreo;
import choreo.Choreo.TrajectoryCache;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.scoreL1;
import frc.robot.commands.scoreL4;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.SuperStructureSubsystem;

public class BlueMid extends SequentialCommandGroup{
    private final SuperStructureSubsystem superStruc;
    private final scoreL4 scorel4;
    private final scoreL1 scorel1;
;    private final CommandSwerveDrivetrain swerve;
    private final AutoFactory autoFactory;
    public BlueMid(SuperStructureSubsystem _superStruct,CommandSwerveDrivetrain _swerve,scoreL4 _scoreL4,scoreL1 _scoreL1, AutoFactory _autoFactory){
        superStruc = _superStruct;
        swerve = _swerve;
        scorel4 = _scoreL4;
        scorel1 = _scoreL1;
        autoFactory = _autoFactory;
        TrajectoryCache cache =  autoFactory.cache();
        
          var tr = cache.loadTrajectory("BSM-BH.traj");
         addCommands(
            Commands.runOnce(()-> { swerve.setTrajectory( (Trajectory<SwerveSample>)tr.get()); })
        );

    }
    
    }
   /*  public AutoRoutine run(){
        AutoRoutine routine = autoFactory.newRoutine("run");
        AutoTrajectory BSMBH = routine.trajectory("BSM-BH.traj");
        AutoTrajectory BHBLS = routine.trajectory("BH-BLS.traj");

        routine.active().onTrue(BSMBH.cmd());
        BSMBH.done().onTrue(scorel4);
        if(scorel4.isFinished() ) BHBLS.cmd();
        
        return routine;
    }*/

