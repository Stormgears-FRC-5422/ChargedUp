// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
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
import frc.robot.commands.trajectory.FollowPathCommand;
import frc.robot.commands.trajectory.Paths;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.Compression;
import frc.robot.commands.GyroCommand;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.subsystems.drive.DrivetrainFactory;

import frc.robot.subsystems.drive.IllegalDriveTypeException;
import frc.robot.subsystems.stormnet.StormNet;
import frc.utils.joysticks.StormLogitechController;

import java.util.HashMap;

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
    Compression m_compression;

    StormNet m_stormNet;

    StormLogitechController m_controller;

    private SendableChooser<Paths.AutoPath> autoPathChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() throws IllegalDriveTypeException {

        if (usePneumatics) {
            m_compression = new Compression();
        }

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
                autoPathChooser = new SendableChooser<>();
                autoPathChooser.setDefaultOption(Paths.straightPath.name,
                        new Paths.AutoPath(Paths.straightPath.name, Paths.straight180Path.path));
                for (Paths.PathWithName path: Paths.listOfPaths) {
                    var autoPath = new Paths.AutoPath(path.name, path.path);
                    autoPathChooser.addOption(path.name, autoPath);
                }
                SmartDashboard.putData("Auto Paths", autoPathChooser);
            }
        } else {
            System.out.println("NOT using drive");
        }

        if (usePneumatics) {
            m_compression = new Compression();
        }

        // TODO - how do we know that this worked? e.g. what fails if the joystick is unplugged?
        if (useController) {
            m_controller = new StormLogitechController(kLogitechControllerPort);
            System.out.println("using controller");
        } else {
            System.out.println("NOT using controller");
        }

        if (useStormNet) {
          StormNet.init();
          m_stormNet = StormNet.getInstance();
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
            // SlewRateLimiter forwardInputLimiter = new SlewRateLimiter(1);
            // SlewRateLimiter sidewaysInputLimiter = new SlewRateLimiter(1);
            // SlewRateLimiter rotationInputLimiter = new SlewRateLimiter(1);

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
            var commandPlayer = Shuffleboard.getTab("Path Following");

            HashMap<String, Command> commandHashMap = new HashMap<>();
            commandHashMap.put("halfway", new PrintCommand("Passed Halfway!"));
            FollowPathWithEvents pathWithEvents = new FollowPathWithEvents(
                    new FollowPathCommand(Paths.straightPath, m_drivetrain),
                    Paths.straightPath.path.getMarkers(),
                    commandHashMap
            );

            SendableChooser<Paths.PathWithName> pathChooser = new SendableChooser<>();
            pathChooser.setDefaultOption(Paths.straightPath.name, Paths.straightPath);
            for (Paths.PathWithName path : Paths.listOfPaths) {
                pathChooser.addOption(path.name, path);
            }

            //log chooser
            commandPlayer.add("Paths", pathChooser)
                    .withWidget(BuiltInWidgets.kComboBoxChooser)
                    .withPosition(0, 0)
                    .withSize(2, 1);
            commandPlayer.add("Run Path Following Command",
                    new FollowPathCommand(pathChooser.getSelected(), m_drivetrain))
                    .withWidget(BuiltInWidgets.kCommand)
                    .withPosition(2, 0)
                    .withSize(2, 1);
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

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        if (useDrive && driveType.equals("SwerveDrive")) {
            var selectedPath = autoPathChooser.getSelected();
            m_robotState.setStartPose(selectedPath.startPose);
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
}

