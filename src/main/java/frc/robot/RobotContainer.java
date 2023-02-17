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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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
                m_robotState.setStartPose(new Pose2d());
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

            var straightPath = PathPlanner.loadPath("Straight Path", 1, 0.5);
            var straight180Path = PathPlanner.loadPath("180 while forward", 1, 0.5);
            var tPath = PathPlanner.loadPath("TPath", 1, 0.5);
            var circularPath = PathPlanner.loadPath("Circular", 1, 0.5);
            var Auto1 = PathPlanner.loadPath("Auto1", 1, 0.5);
            var testPath = PathPlanner.loadPath("Test", 1, 0.5);

            SmartDashboard.putData("180 while going forward path",
                    getPathFollowCommand(straight180Path));
            SmartDashboard.putData("Circular path",
                    getPathFollowCommand(circularPath));
            SmartDashboard.putData("Auto1 Path (have more space!)",
                    getPathFollowCommand(Auto1));
            SmartDashboard.putData("Test Path (have space!)",
                    getPathFollowCommand(testPath));
            SmartDashboard.putData("Straight Path",
                    getPathFollowCommand(straightPath));
            SmartDashboard.putData("TPath",
                    getPathFollowCommand(tPath));

//            SmartDashboard.putData("Straight Path using our command",
//                    new FollowPathCommand(straightPath, m_drivetrain));
            SmartDashboard.putData("Straight 180 Path using our command",
                    new FollowPathCommand(straight180Path, m_drivetrain));
            SmartDashboard.putData("TPath using our command",
                    new FollowPathCommand(tPath, m_drivetrain));
            SmartDashboard.putData("Circular using our command",
                    new FollowPathCommand(circularPath, m_drivetrain));
            SmartDashboard.putData("Test Path using our command (use caution!)",
                    new FollowPathCommand(testPath, m_drivetrain));
            SmartDashboard.putData("Auto1 using our command",
                    new FollowPathCommand(Auto1, m_drivetrain));

            HashMap<String, Command> commandHashMap = new HashMap<>();
            commandHashMap.put("halfway", new PrintCommand("Passed Halfway!"));

            FollowPathWithEvents pathWithEvents = new FollowPathWithEvents(
                    getPathFollowCommand(straightPath),
                    straightPath.getMarkers(),
                    commandHashMap
            );

            SmartDashboard.putData("Straight Path using our command", new FollowPathCommand(straightPath, m_drivetrain));

            // Simple path with holonomic rotation. Stationary start/end. Max velocity of 4 m/s and max accel of 3 m/s^2
            PathPlannerTrajectory traj2 = PathPlanner.generatePath(
                    new PathConstraints(1, 0.5),
                    new PathPoint(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)), // position, heading(direction of travel), holonomic rotation
                    new PathPoint(new Translation2d(3.0, 1.5), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(90))// position, heading(direction of travel), holonomic rotation
            );

            SmartDashboard.putData("Diagonal", getPathFollowCommand(traj2));
        }
    }

    private PPSwerveControllerCommand getPathFollowCommand(PathPlannerTrajectory path) {
        return new PPSwerveControllerCommand(
                path,
                RobotState.getInstance()::getCurrentPose,
                new PIDController(1., 0, 0),
                new PIDController(1., 0, 0),
                new PIDController(1., 0, 0),
                speeds -> m_drivetrain.drive(speeds, true),
                false,
                m_drivetrain
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return new InstantCommand(() -> System.out.println("Autonomous"));
    }

    void onEnable() {
        m_robotState.onEnable();
        m_drivetrain.onEnable();
        m_poseEstimator.onEnable();
        m_robotState.setStartPose(new Pose2d());
        System.out.println("-------------enabled-------------");
    }

    void onDisable() {
        m_robotState.onDisable();
        m_poseEstimator.onDisable();
        System.out.println("-----------disabled------------");
    }
}

