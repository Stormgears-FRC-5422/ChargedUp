// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.LayoutType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveWithJoystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TrapezoidMoveForward;
import frc.robot.commands.trajectory.FollowPathCommand;
import frc.robot.commands.trajectory.FollowTrajectoryCommand;
import frc.robot.commands.trajectory.Trajectories;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.Compression;
import frc.robot.commands.GyroCommand;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.subsystems.drive.DrivetrainFactory;

import frc.robot.subsystems.drive.IllegalDriveTypeException;
import frc.utils.joysticks.StormLogitechController;

import java.nio.file.Path;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import static frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    RobotState m_robotState;
    GyroCommand m_gyrocommand;
    DrivetrainBase m_drivetrain;
    //    TrapezoidMoveForward trapezoidMoveForwardCommand = new TrapezoidMoveForward(m_drivetrain, 20, 1, 0.2);
    PoseEstimator m_poseEstimator;
    public Compression compressor;

    StormLogitechController m_controller;

    private SendableChooser<PathPlannerTrajectory> PathChooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() throws IllegalDriveTypeException {

        if (useCompressor) compressor = new Compression();

        m_robotState = RobotState.getInstance();
        m_robotState.setStartPose(new Pose2d());

        // Note the pattern of attempting to create the object then disabling it if that creation fails
        if (useDrive) {
            try {
                m_drivetrain = DrivetrainFactory.getInstance(driveType);
            } catch (Exception e) {
                e.printStackTrace();
                useDrive = false;
                System.out.println("NOT using drive - caught exception!");
            }
        } else {
            System.out.println("NOT using drive");
        }
        // Note the pattern of attempting to create the object then disabling it if that creation fails
        if (useDrive) {
            try {
                m_drivetrain = DrivetrainFactory.getInstance(driveType);
            } catch (Exception e) {
                e.printStackTrace();
                useDrive = false;
                System.out.println("NOT using drive - caught exception!");
            }

            if (driveType.equals("SwerveDrive")) {
                m_poseEstimator = new PoseEstimator(
                        m_drivetrain.getSwerveDriveKinematics(),
                        m_drivetrain.getGyroscopeRotation(),
                        m_drivetrain.getSwerveModulePositions());

                //add paths to chooser
                PathChooser.setDefaultOption("Straight Path", Paths.straight180Path);
                PathChooser.addOption("Straight 180 Path", Paths.straight180Path);
                PathChooser.addOption("Diagonal Path", Paths.diagonalPath);
                PathChooser.addOption("Circular Path", Paths.circularPath);
                PathChooser.addOption("T Path", Paths.tPath);
                PathChooser.addOption("Test Path (caution!)", Paths.testPath);
                PathChooser.addOption("Auto1 Path (caution!)", Paths.Auto1);
                SmartDashboard.putData("Auto Paths", PathChooser);
            }
        } else {
            System.out.println("NOT using drive");
        }

        if (useCompressor) {
            compressor = new Compression();
        }

        // TODO - how do we know that this worked? e.g. what fails if the joystick is unplugged?
        if (useController) {
            m_controller = new StormLogitechController(kLogitechControllerPort);
            System.out.println("using controller");
        } else {
            System.out.println("NOT using controller");
        }

        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {

        if (useDrive && useController) {
            // TODO - get rid of the magic numbers here and make these config settings (do we have them already?)
            SlewRateLimiter forwardInputLimiter = new SlewRateLimiter(1);
            SlewRateLimiter sidewaysInputLimiter = new SlewRateLimiter(1);
            SlewRateLimiter rotationInputLimiter = new SlewRateLimiter(1);

            DriveWithJoystick driveWithJoystick = new DriveWithJoystick(
                    m_drivetrain,
                    () -> m_controller.getWpiXAxis() * kDriveSpeedScale,
                    () -> m_controller.getWpiYAxis() * kDriveSpeedScale,
                    () -> m_controller.getWpiZAxis() * kDriveSpeedScale,
                    () -> m_controller.getRawButton(2));

            m_drivetrain.setDefaultCommand(driveWithJoystick);

            m_gyrocommand = new GyroCommand(m_drivetrain, 180);

            new Trigger(() -> m_controller.getRawButton(1)).onTrue(new InstantCommand(m_drivetrain::zeroGyroscope));
            new Trigger(() -> m_controller.getRawButton(3)).onTrue(new InstantCommand(driveWithJoystick::toggleFieldRelative));
            new Trigger(() -> m_controller.getRawButton(4)).whileTrue(new GyroCommand(m_drivetrain, 180));
            new Trigger(() -> m_controller.getRawButton(5)).onTrue(driveWithJoystick);
        }

        if (useDrive && driveType.equals("SwerveDrive") && useDrive) {
//            SmartDashboard.putData("Trapezoid Move Forward Command",
//                    new TrapezoidMoveForward(m_drivetrain, 5, 1, 0.5));
//
//            Supplier<Trajectory> straightTrajectorySupplier = () -> Trajectories.
//                    straightLineNoTurn(1, 0.2, m_drivetrain.getSwerveDriveKinematics());
//            var straightLineTrajectoryCommand = new FollowTrajectoryCommand(straightTrajectorySupplier, m_drivetrain);
//            SmartDashboard.putData("Straight Line Trajectory", straightLineTrajectoryCommand);
//
//            Supplier<Trajectory> turningTrajectorySupplier = () -> Trajectories.
//                    straightLineWhileTurn(45, 1, 0.2, m_drivetrain.getSwerveDriveKinematics());
//            var turningTrajectoryCommand = new FollowTrajectoryCommand(turningTrajectorySupplier, m_drivetrain);
//            SmartDashboard.putData("Turning in line Trajectory", turningTrajectoryCommand);



            var commandPlayer = Shuffleboard.getTab("Path Following Commands");

            var PPSwerveCommandPlayer = commandPlayer.
                    getLayout("PP Swerve Commands", BuiltInLayouts.kGrid)
                    .withPosition(0, 0)
                    .withSize(4, 4);
            PPSwerveCommandPlayer.add("Straight Path",
                    getPathFollowCommand("Straight Path", Paths.straightPath));
            PPSwerveCommandPlayer.add("180 while going forward path",
                    getPathFollowCommand("Straight Path With 180", Paths.straight180Path));
            PPSwerveCommandPlayer.add("Circular path",
                    getPathFollowCommand("Circular Path", Paths.circularPath));
            PPSwerveCommandPlayer.add("Auto1 Path (have more space!)",
                    getPathFollowCommand("Auto1 Path", Paths.Auto1));
            PPSwerveCommandPlayer.add("Test Path (have space!)",
                    getPathFollowCommand("Test Path", Paths.testPath));
            PPSwerveCommandPlayer.add("TPath",
                    getPathFollowCommand("T Path", Paths.tPath));
            PPSwerveCommandPlayer.add("Diagonal Path",
                    getPathFollowCommand(Paths.diagonalPath));

            var FollowPathCommandPlayer = commandPlayer.
                    getLayout("Follow Path Commands", BuiltInLayouts.kGrid)
                    .withPosition(4, 0)
                    .withSize(4, 4);
            FollowPathCommandPlayer.add("Straight our command",
                    new FollowPathCommand(Paths.straightPath, m_drivetrain));
            FollowPathCommandPlayer.add("180 while going forward our command",
                    new FollowPathCommand(Paths.straight180Path, m_drivetrain));
            FollowPathCommandPlayer.add("Circular our command",
                    new FollowPathCommand(Paths.circularPath, m_drivetrain));
            FollowPathCommandPlayer.add("Auto1 (have more space!) our command",
                    new FollowPathCommand(Paths.Auto1, m_drivetrain));
            FollowPathCommandPlayer.add("Test (have space!) our command",
                    new FollowPathCommand(Paths.testPath, m_drivetrain));
            FollowPathCommandPlayer.add("T our command",
                    new FollowPathCommand(Paths.tPath, m_drivetrain));
            FollowPathCommandPlayer.add("Diagonal Path with our command",
                    new FollowPathCommand(Paths.diagonalPath, m_drivetrain));

            HashMap<String, Command> commandHashMap = new HashMap<>();
            commandHashMap.put("halfway", new PrintCommand("Passed Halfway!"));
            FollowPathWithEvents pathWithEvents = new FollowPathWithEvents(
                    new FollowPathCommand(Paths.straightPath, m_drivetrain),
                    Paths.straightPath.getMarkers(),
                    commandHashMap
            );
            commandPlayer.add("Straight Path With Events", pathWithEvents).withPosition(5, 0);

        }
    }

    private SequentialCommandGroup getPathFollowCommand(String pathName, PathPlannerTrajectory path) {
        return
                new PrintCommand("Path Name: " + pathName).andThen(
                new PrintCommand("States: " + path)).andThen(
                new PrintCommand("Start State: " + path.getInitialState())).andThen(
                new PrintCommand("Middle State: " + path.getState(path.getStates().size()/2))).andThen(
                new PrintCommand("End State: " + path.getEndState())).andThen(
                new PrintCommand("Pose at Start: " + m_robotState.getCurrentPose())).andThen(
                new PrintCommand("Time at Start: " + m_robotState.getTimeSeconds())).andThen(
                new PPSwerveControllerCommand(
                    path,
                    m_robotState::getCurrentPose,
                    new PIDController(3.0, 0, 0),
                    new PIDController(3.0, 0, 0),
                    new PIDController(1.0, 0, 0),
                    speeds -> m_drivetrain.drive(speeds, true),
                    false,
                    m_drivetrain)
                ).andThen(
                new PrintCommand("Pose at End: " + m_robotState.getCurrentPose())).andThen(
                new PrintCommand("Time at End: " + m_robotState.getTimeSeconds()));
    }

    private SequentialCommandGroup getPathFollowCommand(PathPlannerTrajectory path) {
        return getPathFollowCommand("no name", path);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        if (useDrive && driveType.equals("SwerveDrive")) {
            var selectedPath = PathChooser.getSelected();
            m_robotState.setStartPose(selectedPath.getInitialPose());
            return new FollowPathCommand(selectedPath, m_drivetrain);
        }
        return new PrintCommand("Autonomous! -----");
    }

    void onEnable() {
        m_robotState.onEnable();
        m_drivetrain.onEnable();
        m_poseEstimator.onEnable();
        System.out.println("-------------enabled-------------");
    }

    void onDisable() {
        m_robotState.onDisable();
        m_poseEstimator.onDisable();
        System.out.println("-----------disabled------------");
    }

    private static final class Paths {
        public static PathPlannerTrajectory straightPath = PathPlanner.loadPath("Straight Path", 1, 0.5);
        public static PathPlannerTrajectory straight180Path = PathPlanner.loadPath("180 while forward", 1, 0.5);
        public static PathPlannerTrajectory tPath = PathPlanner.loadPath("TPath", 3, 1);
        public static PathPlannerTrajectory circularPath = PathPlanner.loadPath("Circular", 1, 0.5);
        public static PathPlannerTrajectory Auto1 = PathPlanner.loadPath("Auto1", 1, 0.5);
        public static PathPlannerTrajectory testPath = PathPlanner.loadPath("Test", 1, 0.5);

        public static PathPlannerTrajectory diagonalPath = PathPlanner.generatePath(
                new PathConstraints(1, 0.7),
                new PathPoint(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)), // position, heading(direction of travel), holonomic rotation
                new PathPoint(new Translation2d(3.0, 0.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180))// position, heading(direction of travel), holonomic rotation
        );
    }
}

