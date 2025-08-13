// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.util.Units;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.pathfinding.*;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.auto.CommandUtil;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Claw.ClawSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.commands.Intake;
import frc.robot.commands.scoreL1;
import frc.robot.commands.simDemo;
import frc.robot.commands.L1;
import frc.robot.commands.L2;
import frc.robot.commands.L3;
import frc.robot.commands.scoreL4;
import frc.robot.commands.Zero;

public class RobotContainer {
        Pose2d blueStartPillar = new Pose2d(8.0,4.6,new Rotation2d(0));
        Pose2d blueStartMid = new Pose2d(8.0,6.1,new Rotation2d(0));
        Pose2d blueStartWall = new Pose2d(8.0,7.2,new Rotation2d(0));

        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                      // speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
        private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        private final Telemetry logger = new Telemetry(MaxSpeed);

        private final CommandXboxController joystick = new CommandXboxController(0);
        private final CommandXboxController coJoystick = new CommandXboxController(1);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        public final ElevatorSubsystem elevator = new ElevatorSubsystem();
        public final ClawSubsystem claw = new ClawSubsystem();
        /* Path follower */
        private final SendableChooser<Command> autoChooser;

        Pose2d targetPose = new Pose2d(1.513, 7.332, Rotation2d.fromDegrees(-53));

        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                        3.0, 4.0,
                        Units.degreesToRadians(1000), Units.degreesToRadians(1000));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
                        targetPose,
                        constraints,
                        0.0 // Goal end velocity in meters/sec
                            // Rotation delay distance in meters. This is how far the robot should travel
                            // before attempting to rotate.
        );

        scoreL1 scorel1 = new scoreL1(elevator, claw);
        L1 l1;
        L2 l2;
        L3 l3;
        scoreL4 scorel4 = new scoreL4(elevator, claw);
        Zero zero;
        // You could technically run the demo on robot but im scared....
        simDemo demo = new simDemo(elevator, claw);
        Intake intake;
        double finalH = 0;
        double finalX = 0;
        double clawAngle = 0;

        public RobotContainer() {
                NamedCommands.registerCommand("Score L1", scorel1);
                NamedCommands.registerCommand("pathFind", pathfindingCommand);
                NamedCommands.registerCommand("Score L4", demo);
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Mode", autoChooser);
                configureBindings();

        }

        private void configureBindings() {
                drivetrain.setDefaultCommand(
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(-joystick.getLeftY() * MaxSpeed) // Negative Y(forward)
                                                .withVelocityY(-joystick.getLeftX() * MaxSpeed)// Negative X(left)
                                                .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // negative
                                                                                                            // X(counterclockwise)
                                ));
                joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
                joystick.b().whileTrue(drivetrain.applyRequest(
                                () -> point.withModuleDirection(
                                                new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));
                joystick.pov(0).whileTrue(
                                drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
                joystick.pov(180)
                                .whileTrue(drivetrain.applyRequest(
                                                () -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                /*
                 * joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction
                 * .kForward));
                 * joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction
                 * .kReverse));
                 * joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(
                 * Direction.kForward));
                 * joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(
                 * Direction.kReverse));
                 */
                // reset the field-centric heading on left bumper press
                joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                drivetrain.registerTelemetry(logger::telemeterize);
                coJoystick.b().onTrue(demo);// 20
                coJoystick.back().onChange(Commands.runOnce(() -> claw.setAxis(0.0)));

                coJoystick.x().onChange(Commands.runOnce(() -> claw.setAxis(0.698)));// 40
                coJoystick.a().onChange(Commands.runOnce(() -> claw.setAxis(2.61)));// 150
                coJoystick.y().onChange(Commands.runOnce(() -> elevator.setHeight(5.3)));

        }

        public void periodic() {
                clawAngle = claw.lastKnownAngle;
                if (clawAngle == 0) {
                        finalH = 0;
                        finalX = 0;
                } else if (clawAngle == 0.349) {
                        finalH = 0.115;// 0.1415;
                        finalX = -0.11;// -0.13;

                } else if (clawAngle == 0.698) {
                        finalH = 0.259;
                        finalX = -0.1755;

                } else if (clawAngle == 2.61) {
                        finalH = 0.825;// 0.72;
                        finalX = 0.31;

                }
                // using a sin reggression model translation should be
                // double modelY = 0.547343 * Math.sin((10.77753 * claw.sim.getAngleRads()) -
                // 0.466346) + 0.821554;
                // double modelX = Math.asin(claw.sim.getAngleRads() - 0.821554 / 0.547343) +
                // 0.466346 / 10.77753;
                // this was wrong:sob:

                Logger.recordOutput("Sim/Drivetrain Pose", new Pose3d(drivetrain.getState().Pose));
                Logger.recordOutput("Sim/mech2d/elevator", elevator.elevatorMech);
                Logger.recordOutput("Sim/Final Position", new Pose3d[] {
                                new Pose3d(0, 0, elevator.elevatorSim.getPositionMeters(),
                                                new Rotation3d(0, 0, 0)),
                                new Pose3d(0, 0, elevator.carriagElevatorSim.getPositionMeters(),
                                                new Rotation3d(0, 0, 0)),
                                new Pose3d(finalX, 0, elevator.carriagElevatorSim.getPositionMeters() + finalH,
                                                new Rotation3d(0, claw.sim.getAngleRads(), 0))
                });
                Logger.recordOutput("calc", new Pose3d[] {
                                new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)),
                                new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)),
                                new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)),
                });

        }

        public Command getAutonomousCommand() {
                /* Run the path selected from the auto chooser */
                 return autoChooser.getSelected();               

        }
}
