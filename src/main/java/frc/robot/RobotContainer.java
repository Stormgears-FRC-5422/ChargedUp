// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.commands.arm.BasicArm;
import frc.robot.commands.trajectory.FollowPathCommand;
import frc.robot.commands.trajectory.Paths;
import frc.robot.subsystems.IEnabledDisabled;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.stormnet.StormNet;
import frc.robot.subsystems.Compression;
import frc.robot.commands.GyroCommand;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.subsystems.drive.DrivetrainFactory;

import frc.robot.subsystems.drive.IllegalDriveTypeException;
import frc.utils.joysticks.StormLogitechController;
import frc.utils.joysticks.StormXboxController;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    // **********
    // SUBSYSTEMS
    // **********
    DrivetrainBase m_drivetrain;
    Compression m_compression;
    Arm m_arm;
    StormNet m_stormNet;
    NavX m_navX;

    //List of subsystems to be enabled and disabled
    List<IEnabledDisabled> m_enabledAndDisabledSystems;

    // **********
    // COMMANDS
    // **********
    GyroCommand m_gyrocommand;
    BasicArm m_basicArm;
    //    TrapezoidMoveForward trapezoidMoveForwardCommand = new TrapezoidMoveForward(m_drivetrain, 20, 1, 0.2);

    // **********
    // Other
    // **********
    RobotState m_robotState;
    PoseEstimator m_poseEstimator;

    StormLogitechController m_controller;

    private final SendableChooser<Paths.PathWithName> autoPathChooser = new SendableChooser<>();
    private final Map<String, Command> autoEventMap = new HashMap<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() throws IllegalDriveTypeException {

        m_robotState = RobotState.getInstance();
        m_enabledAndDisabledSystems.add(m_robotState);

        if (useNavX) {
            m_navX = new NavX();
            m_enabledAndDisabledSystems.add(m_navX);
        } else
            System.out.println("NOT using navX");

        // Note the pattern of attempting to create the object then disabling it if that creation fails
        if (useDrive) {
            try {
                m_drivetrain = DrivetrainFactory.getInstance(driveType);
                m_enabledAndDisabledSystems.add(m_drivetrain);
            } catch (Exception e) {
                e.printStackTrace();
                 useDrive = false;
                System.out.println("NOT using drive - caught exception!");
            }

            if (driveType.equals("SwerveDrive")) {
                m_poseEstimator = new PoseEstimator(
                        m_drivetrain.getSwerveDriveKinematics(),
                        m_drivetrain::getSwerveModulePositions);
                m_enabledAndDisabledSystems.add(m_poseEstimator);

                autoEventMap.put("PickUpFromGround", new PrintCommand("Picking up game piece from ground!"));
                autoEventMap.put("StowArm", new PrintCommand("Stowing the arm!"));
                autoEventMap.put("LiftArm", new PrintCommand("Lifting the arm!"));
                autoEventMap.put("PlaceGamePiece", new PrintCommand("Placing the game piece!"));
                autoEventMap.put("Balance", new PrintCommand("Balancing on charging station!"));

                //add paths to chooser
                for (var pathWithName : Paths.listOfPaths) {
                    autoPathChooser.addOption("Auto " + pathWithName.name, pathWithName);
                }
                SmartDashboard.putData("Auto Paths", autoPathChooser);
            }
        } else {
            System.out.println("NOT using drive");
        }

        if (useArm) {
            m_arm = new Arm();
            m_enabledAndDisabledSystems.add(m_arm);
        } else {
            System.out.println("NOT using arm");
        }

        if (usePneumatics) {
            m_compression = new Compression();
        } else
            System.out.println("NOT using pneumatics");

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
        } else
            System.out.println("NOT using stormnet");

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
        StormXboxController xboxController = new StormXboxController(1);

        if (useArm && useController) {
            m_basicArm = new BasicArm(m_arm,
                    xboxController::getLeftJoystickY,
                    xboxController::getRightJoystickY);
            m_arm.setDefaultCommand(m_basicArm);
        }

        if (usePneumatics && useController) {
            new Trigger(xboxController::getYButtonIsHeld).onTrue(new InstantCommand(m_compression::grabCone));
            new Trigger(xboxController::getXButtonIsHeld).onTrue(new InstantCommand(m_compression::grabCube));
            new Trigger(xboxController::getBButtonIsHeld).onTrue(new InstantCommand(m_compression::release));
        } else {
            System.out.println("Pneumatics or controller not operational");
        }

        if (useDrive && useController) {
            // TODO - get rid of the magic numbers here and make these config settings (do we have them already?)
            SlewRateLimiter forwardInputLimiter = new SlewRateLimiter(1);
            SlewRateLimiter sidewaysInputLimiter = new SlewRateLimiter(1);
            SlewRateLimiter rotationInputLimiter = new SlewRateLimiter(1);

	        DriveWithJoystick driveWithJoystick = new DriveWithJoystick(
                    m_drivetrain,
                    m_controller::getWpiXAxis,
                    m_controller::getWpiYAxis,
                    m_controller::getWpiZAxis,
                    () -> m_controller.getRawButton(2));

    	    m_drivetrain.setDefaultCommand(driveWithJoystick);

        	m_gyrocommand = new GyroCommand(m_drivetrain, 180);

        	new Trigger(() -> m_controller.getRawButton(1)).onTrue(new InstantCommand(m_navX::zeroYaw));
        	new Trigger(() -> m_controller.getRawButton(3)).onTrue(new InstantCommand(driveWithJoystick::toggleFieldRelative));
        	new Trigger(() -> m_controller.getRawButton(4)).whileTrue(new GyroCommand(m_drivetrain, 180));
            new Trigger(() -> m_controller.getRawButton(5)).onTrue(new InstantCommand(() -> {
                m_drivetrain.getCurrentCommand().cancel();
                driveWithJoystick.schedule();
            }));
    }


        if (useDrive && driveType.equals("SwerveDrive")) {

            var commandPlayer = Shuffleboard.getTab("Path Following Commands");

            SendableChooser<Command> pathCommandChooser = new SendableChooser<>();

            for (var path : Paths.listOfPaths) {
                pathCommandChooser.addOption(path.name, getPathFollowCommand(path.name, path.path));
            }

            HashMap<String, Command> commandHashMap = new HashMap<>();
            commandHashMap.put("halfway", new PrintCommand("Passed Halfway!"));
            FollowPathWithEvents pathWithEvents = new FollowPathWithEvents(
                    getPathFollowCommand("Straight Path with markers", Paths.straightPath),
                    Paths.straightPath.getMarkers(),
                    commandHashMap
            );

            pathCommandChooser.addOption("Straight with events", pathWithEvents);
            pathCommandChooser.addOption("Team Number", Paths.getTeamNumberPathCommand(m_drivetrain));

            pathCommandChooser.addOption("Straight Path from robot",
                    getPathFollowCommand("Straight path from robot",
                            Paths.getPathFromRobotPose(
                                    new Pose2d(2, 0, Rotation2d.fromDegrees(0)),
                                    2, 3)));

            commandPlayer
                    .add("Path Chooser", pathCommandChooser)
                    .withWidget(BuiltInWidgets.kComboBoxChooser)
                    .withSize(2, 1).withPosition(0, 0);

            commandPlayer
                    .add("Run Selected Command", pathCommandChooser.getSelected())
                    .withWidget(BuiltInWidgets.kCommand)
                    .withSize(2, 1).withPosition(2, 0);
        }
    }

    private Command getPathFollowCommand(String message, PathPlannerTrajectory path) {
        return new SequentialCommandGroup(
                new PrintCommand(message),
                new FollowPathCommand(path, m_drivetrain)
        );
    }

    private Command getPathFollowCommand(PathPlannerTrajectory path) {
        return getPathFollowCommand("No message", path);
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
            return new FollowPathWithEvents(
                    getPathFollowCommand("Auto path starting " + selectedPath.name, selectedPath.path),
                    selectedPath.path.getMarkers(),
                    autoEventMap
            );
        }
        return new PrintCommand("Autonomous! -----");
    }


    void onEnable() {
        for (var system : m_enabledAndDisabledSystems) {
            system.onEnable();
        }
        System.out.println("-------------enabled-------------");
    }

    void onDisable() {
        for (var system : m_enabledAndDisabledSystems) {
            system.onDisable();
        }
        System.out.println("-----------disabled------------");
    }
}

