// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.Consumer;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGWriter.AdvantageScopeOpenBehavior;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.SuperStructureSubsystem;
import frc.robot.subsystems.Claw.ClawSubsystem;
import frc.robot.subsystems.Claw.feed.FeedIO;
import frc.robot.subsystems.Claw.feed.FeedRevMaxIO;
import frc.robot.subsystems.Claw.ClawIO;
import frc.robot.subsystems.Claw.ClawTalonFXIO;
import frc.robot.subsystems.Claw.ClawSubsystem.ClawStates;
import frc.robot.subsystems.Claw.ClawSubsystem.FeedStates;
import frc.robot.subsystems.SuperStructureSubsystem.superState;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorTalonFXIO;

@SuppressWarnings("unused")
public class RobotContainer {
        Pose2d blueStartPillar = new Pose2d(8.0, 5, new Rotation2d(3.14));
        Pose2d blueStartMid = new Pose2d(8.0, 6.1, new Rotation2d(3.14));
        Pose2d blueStartWall = new Pose2d(8.0, 7.2, new Rotation2d(3.14));
        Pose2d redStartPillar = new Pose2d(9.4, 3, new Rotation2d(0));
        Pose2d redStartMid = new Pose2d(9.3, 1.9, new Rotation2d(0));
        Pose2d redStartWall = new Pose2d(9.15, 0.8, new Rotation2d(0));
        public Pose2d blueTargetPose = new Pose2d(1.513, 7.332, Rotation2d.fromDegrees(-53));
        public Pose2d redTargetPose = new Pose2d(15.965, 0.6, Rotation2d.fromDegrees(53));
        public Pose2d targetPose;
        public Pose2d startingPose;
        boolean algeaMode = false;

        public Alliance currentAlliance = Alliance.Blue;

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

        public superState state;
        public final ElevatorSubsystem elevator;
        public final ClawSubsystem claw;
        private final SuperStructureSubsystem superStruct;

        /* Path follower */
        private final SendableChooser<Command> autoChooser;
        private final SendableChooser<String> poseChooser = new SendableChooser<String>();
        // SendableBuilder poseBuilder;
        String _chosenPose;

        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                        3.0, 4.0,
                        Units.degreesToRadians(300), Units.degreesToRadians(300));

        Command pathfindingCommand;

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command bluePathfindingCommand = AutoBuilder.pathfindToPose(
                        blueTargetPose,
                        constraints,
                        0.0 // Goal end velocity in meters/sec
                            // Rotation delay distance in meters. This is how far the robot should travel
                            // before attempting to rotate.
        );
        Command redPathfindingCommand = AutoBuilder.pathfindToPose(
                        redTargetPose,
                        constraints,
                        0.0 // Goal end velocity in meters/sec
                            // Rotation delay distance in meters. This is how far the robot should travel
                            // before attempting to rotate.
        );

        private Command controllerRumbleCommand() {
                return Commands.startEnd(
                                () -> {
                                        joystick.getHID().setRumble(RumbleType.kBothRumble, 1.0);
                                        coJoystick.getHID().setRumble(RumbleType.kBothRumble, 1.0);
                                },
                                () -> {
                                        joystick.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                                        coJoystick.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                                });
        }

        /*
         * scoreL1 scorel1 = new scoreL1(elevator, claw);
         * L1 l1;
         * L2 l2;
         * L3 l3;
         * scoreL4 scorel4 = new scoreL4(elevator, claw);
         * testL1 test2 = new testL1(elevator, claw);
         * Zero zero;
         * // You could technically run the demo on robot but im scared....
         * simDemo demo = new simDemo(elevator, claw);
         */
        double finalH = 0;
        double finalX = 0;
        double clawAngle = 0;

        public RobotContainer() {
                DriverStation.getAlliance().ifPresent(currentAlliance -> {
                });

                if (currentAlliance == Alliance.Blue) {
                        NamedCommands.registerCommand("pathFind", bluePathfindingCommand);
                } else {
                        NamedCommands.registerCommand("pathFind", redPathfindingCommand);

                }

                poseChooser.addOption("Collum", "Collum");
                poseChooser.addOption("Middle", "Middle");
                poseChooser.addOption("Wall", "Wall");
                poseChooser.setDefaultOption("Middle", "Middle");
                NamedCommands.registerCommand("Score L1", Commands.none());
                NamedCommands.registerCommand("Score L4", Commands.none());
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Mode", autoChooser);

                state = superState.Home;
                elevator = new ElevatorSubsystem(new ElevatorTalonFXIO());
                claw = new ClawSubsystem(new ClawTalonFXIO(), new FeedRevMaxIO());
                superStruct = new SuperStructureSubsystem(elevator, claw, drivetrain);
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

                /*
                 * coJoystick.a()
                 * .onChange(Commands.runOnce(() -> {
                 * superStruct.changeState(superState.Home);
                 * }, superStruct));
                 */

                coJoystick.y()
                                .onChange(Commands.runOnce(() -> {
                                        superStruct.changeState(superState.L4);
                                }, superStruct));
                coJoystick.start()
                                .toggleOnTrue(Commands.runOnce(() -> {
                                        superStruct.changeState(superState.Intake);
                                }, superStruct)).toggleOnFalse(Commands.runOnce(() -> {
                                        superStruct.changeState(superState.Home);
                                }, superStruct)).debounce(0.5);
                coJoystick.rightBumper()
                                .onTrue(Commands.runOnce(() -> {
                                        superStruct.changeState(FeedStates.Outake);
                                }, superStruct).alongWith(controllerRumbleCommand().withTimeout(0.5)))
                                .onFalse(Commands.runOnce(() -> {
                                        superStruct.changeState(FeedStates.PercentOut, 0.0);
                                }, superStruct));
                coJoystick.leftBumper()
                                .onTrue(Commands.runOnce(() -> {
                                        superStruct.changeState(FeedStates.PercentOut, 0.2);
                                }, superStruct).alongWith(controllerRumbleCommand().withTimeout(0.5)))
                                .onFalse(Commands.runOnce(() -> {
                                        superStruct.changeState(FeedStates.PercentOut, 0.0);
                                }, superStruct));
                coJoystick.povRight().onChange(Commands.runOnce(()-> {
                        superStruct.changeState(superState.DeAlgea_L2);
                } , superStruct));
                coJoystick.povLeft().onChange(Commands.runOnce(()-> {
                        superStruct.changeState(superState.DeAlgea_L3);
                } , superStruct));
                coJoystick.povDown().onChange(Commands.runOnce(()-> {
                        superStruct.changeState(superState.Processor);
                } , superStruct));
                coJoystick.x().onChange(Commands.runOnce(() -> {

                        superStruct.changeState(superState.L3);

                }, superStruct));
                coJoystick.b().onChange(Commands.runOnce(() -> {

                        superStruct.changeState(superState.L2);

                }, superStruct));
                coJoystick.a().onChange(Commands.runOnce(() -> {

                        superStruct.changeState(superState.Home);

                }, superStruct));

        }

        public void positionStartup() {
                _chosenPose = poseChooser.getSelected();

                switch (_chosenPose) {
                        case "Wall":
                                if (currentAlliance == Alliance.Blue) {
                                        startingPose = blueStartWall;
                                } else {
                                        startingPose = redStartWall;
                                }
                                break;
                        case "Middle":
                                if (currentAlliance == Alliance.Blue) {
                                        startingPose = blueStartMid;
                                } else {
                                        startingPose = redStartMid;
                                }
                                break;
                        case "Collum":
                                if (currentAlliance == Alliance.Blue) {
                                        startingPose = blueStartPillar;
                                } else {
                                        startingPose = redStartPillar;
                                }
                                break;
                        default:
                                break;
                }
        }

        public void periodic() {
                DriverStation.getAlliance().ifPresent(currentAlliance -> {
                        SmartDashboard.putString("Ali", currentAlliance.toString());

                });
                SmartDashboard.putData(poseChooser);

                // Logger.recordOutput("Alli", currentAlliance.toString());
                Logger.recordOutput("containstate", state);
                if (DriverStation.isDisabled()) {
                        positionStartup();
                        drivetrain.resetPose(startingPose);
                }

                /*
                 * clawAngle = claw.lastKnownAngle;
                 * if (clawAngle == 0) {
                 * finalH = 0;
                 * finalX = 0;
                 * } else if (clawAngle == 0.349) {
                 * finalH = 0.115;// 0.1415;
                 * finalX = -0.11;// -0.13;
                 * 
                 * } else if (clawAngle == 0.698) {
                 * finalH = 0.259;
                 * finalX = -0.1755;
                 * 
                 * } else if (clawAngle == 2.61) {
                 * finalH = 0.825;// 0.72;
                 * finalX = 0.31;
                 * 
                 * }
                 * // using a sin reggression model translation should be
                 * // double modelY = 0.547343 * Math.sin((10.77753 * claw.sim.getAngleRads()) -
                 * // 0.466346) + 0.821554;
                 * // double modelX = Math.asin(claw.sim.getAngleRads() - 0.821554 / 0.547343) +
                 * // 0.466346 / 10.77753;
                 * // this was wrong:sob:
                 * 
                 * Logger.recordOutput("Sim/Drivetrain Pose", new
                 * Pose3d(drivetrain.getState().Pose));
                 * Logger.recordOutput("Sim/mech2d/elevator", elevator.elevatorMech);
                 * Logger.recordOutput("Sim/Final Position", new Pose3d[] {
                 * new Pose3d(0, 0, elevator.elevatorSim.getPositionMeters(),
                 * new Rotation3d(0, 0, 0)),
                 * new Pose3d(0, 0, elevator.carriagElevatorSim.getPositionMeters(),
                 * new Rotation3d(0, 0, 0)),
                 * new Pose3d(finalX, 0, elevator.carriagElevatorSim.getPositionMeters() +
                 * finalH,
                 * new Rotation3d(0, claw.sim.getAngleRads(), 0))
                 * });
                 * Logger.recordOutput("calc", new Pose3d[] {
                 * new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)),
                 * new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)),
                 * new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)),
                 * });
                 */

        }

        public Command getAutonomousCommand() {
                /* Run the path selected from the auto chooser */
                return autoChooser.getSelected();

        }
}
