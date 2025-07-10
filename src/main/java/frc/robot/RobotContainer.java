// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.regex.MatchResult;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Claw.ClawSubsystem;
import frc.robot.subsystems.Claw.ClawSubsystem.clawState;
import frc.robot.commands.Intake;
import frc.robot.commands.scoreL1;
import frc.robot.commands.L1;
import frc.robot.commands.L2;
import frc.robot.commands.L3;
import frc.robot.commands.L4;
import frc.robot.commands.Zero;
import frc.robot.commands.travel;

@SuppressWarnings("unused")

public class RobotContainer {
        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                      // speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second
                                                                                          // max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
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
        private final XboxController cc = new XboxController(2);
        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        public final ElevatorSubsystem elevator = new ElevatorSubsystem();
        public final ClawSubsystem claw = new ClawSubsystem();
        /* Path follower */
        private final SendableChooser<Command> autoChooser;
        PrintCommand notA = new PrintCommand("NO");
        PrintCommand A = new PrintCommand("NO");
        scoreL1 scorel1;
        L1 l1;
        L2 l2;
        L3 l3;
        L4 l4;
        Zero zero;
        Intake intake;

        public RobotContainer() {
                autoChooser = AutoBuilder.buildAutoChooser("Tests");
                SmartDashboard.putData("Auto Mode", autoChooser);
                configureBindings();

        }

        private void configureBindings() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive
                                                                                                                   // forward
                                                                                                                   // with
                                                                                                                   // negative
                                                                                                                   // Y
                                                                                                                   // (forward)
                                                .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with
                                                                                                // negative X (left)
                                                .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive
                                                                                                            // counterclockwise
                                                                                                            // with
                                                                                                            // negative
                                                                                                            // X (left)
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
                // coJoystick.a().onChange(zero);
                coJoystick.b().onChange(Commands.runOnce(() -> claw.setAxis(20.0)));
                coJoystick.back().onChange(Commands.runOnce(() -> claw.setAxis(0.0)));

                coJoystick.x().onChange(Commands.runOnce(() -> claw.setAxis(40.0)));
                coJoystick.a().onChange(Commands.runOnce(() -> claw.setAxis(150.0)));
                coJoystick.y().onChange(Commands.runOnce(() -> elevator.setHeight(5)));

                // coJoystick.y().onChange(l4);
                // coJoystick.start().onChange(intake);

        }
                double finalH =0;
               double finalX =0;

        public void periodic() {
                double p = claw.lastKnownAngle;
                if(p == 0){
                        finalH =0;
                        finalX =0;
                }else if(p == 20){
                        finalH =0.1415;
                        finalX =-0.13;


                }else if(p == 40){
                        finalH =-0.08;
                        finalX =0.318;

                }else if(p == 150){
                        finalH =0.72;
                        finalX =0.335;

                }

                Rotation3d c = new Rotation3d(drivetrain.getState().RawHeading);
                Translation3d t = new Translation3d(drivetrain.getState().Pose.getTranslation());
                Pose3d m = new Pose3d(t, c);
                Pose3d g = new Pose3d(drivetrain.getState().Pose);

                Translation3d q = new Translation3d(drivetrain.getState().Pose.getX(),
                                drivetrain.getState().Pose.getY(), elevator.elevatorSim.getPositionMeters());
                Translation3d carriage = new Translation3d(drivetrain.getState().Pose.getX(),
                                drivetrain.getState().Pose.getY(), elevator.carriagElevatorSim.getPositionMeters());
                // Pose3d q = new Pose3d(-x,-y,elevator.elevatorSim.getPositionMeters(),c);
                // Pose3d carriage = new
                // Pose3d(-x,-y,elevator.carriagElevatorSim.getPositionMeters(),c);
                // using a sin reggression model translation should be
                double modelY = 0.547343 * Math.sin((10.77753 * claw.sim.getAngleRads()) - 0.466346) + 0.821554;
                double modelX = Math.asin(claw.sim.getAngleRads() - 0.821554 / 0.547343) + 0.466346 / 10.77753;
                // this was wrong:sob:
                Pose3d a = new Pose3d(q, c);
                Pose3d b = new Pose3d(carriage, c);
                Logger.recordOutput("myPose", g);
                Logger.recordOutput("Myposearray", a, b);
                Logger.recordOutput("Eout", q);
                Logger.recordOutput("ele", elevator.visElevator);

                System.out.print(modelX);
                Logger.recordOutput("final comp", new Pose3d[] {
                                new Pose3d(0, 0, elevator.elevatorSim.getPositionMeters(), new Rotation3d(0, 0, 0)),
                                new Pose3d(0, 0, elevator.carriagElevatorSim.getPositionMeters(),
                                                new Rotation3d(0, 0, 0)),
                                new Pose3d(finalX, 0, finalH + elevator.carriagElevatorSim.getPositionMeters(), new Rotation3d(0, claw.sim.getAngleRads(), 0))
                });

                Logger.recordOutput("cal", new Pose3d[] {
                                new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)),
                                new Pose3d(0, 0, 0,
                                                new Rotation3d(0, 0, 0)),
                });

        }

        public Command getAutonomousCommand() {
                /* Run the path selected from the auto chooser */
                return autoChooser.getSelected();
        }
}
