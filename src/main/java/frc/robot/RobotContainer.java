// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.DriveWithJoystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.LidarIndicatorCommand;
import frc.robot.commands.arm.ArmCommand;
import frc.robot.commands.arm.BasicArm;
import frc.robot.commands.arm.XYArm;
import frc.robot.commands.trajectory.FollowPathCommand;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.NeoPixel;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.stormnet.StormNet;
import frc.robot.subsystems.Compression;
import frc.robot.commands.GyroCommand;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.subsystems.drive.DrivetrainFactory;

import frc.robot.subsystems.drive.IllegalDriveTypeException;
import frc.utils.joysticks.ButtonBoard;
import frc.utils.joysticks.ButtonBoardConfig;
import frc.utils.joysticks.StormLogitechController;
import frc.utils.joysticks.StormXboxController;

import java.util.function.BooleanSupplier;

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
    NavX m_NavX;
    NeoPixel m_neoPixel;

    // **********
    // COMMANDS
    // **********
    BalanceCommand m_balancecommand;

    GyroCommand m_gyrocommand;
    LidarIndicatorCommand m_lidarIndicatorCommand;
    ArmCommand m_armCommand;
    //    TrapezoidMoveForward trapezoidMoveForwardCommand = new TrapezoidMoveForward(m_drivetrain, 20, 1, 0.2);

    // **********
    // Other
    // **********
    RobotState m_robotState;
    PoseEstimator m_poseEstimator;

    StormLogitechController m_controller;

    ButtonBoardConfig m_buttonboardconfig;


    private SendableChooser<PathPlannerTrajectory> PathChooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() throws IllegalDriveTypeException {

        m_robotState = RobotState.getInstance();
        m_robotState.setStartPose(new Pose2d());

        if (useStatusLights) {
            m_neoPixel = new NeoPixel();
            System.out.println("Using StatusLights");
        } else {
            System.out.println("NOT using StatusLights");
        }

        if (useNavX) {
            m_NavX = new NavX();
            System.out.println("Using NavX");
        } else {
            System.out.println("NOT using NavX");
        }

        // Note the pattern of attempting to create the object then disabling it if that creation fails
        if (useDrive) {
            try {
                m_drivetrain = DrivetrainFactory.getInstance(driveType);
                System.out.println("Successfully created Drivetrain!");
            } catch (Exception e) {
                e.printStackTrace();
                useDrive = false;
                System.out.println("NOT using drive - caught exception!");
            }

            // Need to check useDrive again because we might have failed above
            if (useDrive && driveType.equals("SwerveDrive")) {
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
                System.out.println("Using Drive");
            }
        } else {
            System.out.println("NOT using drive");
        }

        if (useArm) {
            m_arm = new Arm();
            System.out.println("Using arm");
        } else {
            System.out.println("NOT using arm");
        }

        if (usePneumatics) {
            m_compression = new Compression();
            System.out.println("Using Pneumatics");
        } else {
            System.out.println("NOT using Pneumatics");
        }

        // TODO - how do we know that this worked? e.g. what fails if the joystick is unplugged?
        if (useLogitechController) {
            m_controller = new StormLogitechController(kLogitechControllerPort);
            System.out.println("using Logitech controller");
        } else {
            System.out.println("NOT using logitech controller");
        }

        if (useStormNet) {
            System.out.println("Using StormNet");
            StormNet.init();
            m_stormNet = StormNet.getInstance();
            m_stormNet.test();
            var tab = Shuffleboard.getTab("Storm Net");
            tab.addNumber("Lidar Distance", m_stormNet::getLidarDistance);
        } else {
            System.out.println("NOT using StormNet");
        }

        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link CommandPS4Controller
     * PS4} controllers or {@link CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        StormXboxController xboxController;
        System.out.println("useStormNet: " + useStormNet + ", useController: " + useLogitechController);
        if (useStormNet && useLogitechController) {
            //System.out.println("Enabling StormNet test button");
            //new Trigger(() -> m_controller.getRawButton(6)).onTrue(new InstantCommand(m_stormNet::test));
            new Trigger(() -> m_controller.getRawButton(6)).onTrue(new InstantCommand(m_stormNet::getLidarDistance));
        }

        if (useXboxController) {
            xboxController = new StormXboxController(1);

            if (useArm) {
                if (useXYArmMode) {
                    System.out.println("Using XY mode for arm movement");
                    m_armCommand = new XYArm(m_arm,
                            xboxController::getRightJoystickX,
                            xboxController::getLeftJoystickY);
//                    m_armCommand = new XYArm(m_arm,
//                            xboxController::getRightJoystickX,
//                            xboxController::getRightJoystickY);
                } else {
                    System.out.println("Using Angle mode for arm movement");
                    m_armCommand = new BasicArm(m_arm,
                            xboxController::getLeftJoystickY,
                            xboxController::getRightJoystickY);
                }
                m_arm.setDefaultCommand(m_armCommand);
            }

            if (usePneumatics) {
                new Trigger(xboxController::getXButtonIsHeld).onTrue(new InstantCommand(m_compression::grabCubeOrCone));
                new Trigger(xboxController::getBButtonIsHeld).onTrue(new InstantCommand(m_compression::release));
                new Trigger(xboxController::getAButtonIsHeld).onTrue(new InstantCommand(m_compression::stopCompressor));
                new Trigger(xboxController::getYButtonIsHeld).onTrue(new InstantCommand(m_compression::startCompressor));

            } else {
                System.out.println("Pneumatics or controller not operational");
            }
        }

        if (useDrive && useLogitechController) {
            // TODO - get rid of the magic numbers here and make these config settings (do we have them already?)
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

            if (useDrive && driveType.equals("SwerveDrive")) {
                new Trigger(() -> m_controller.getRawButton(7)).whileTrue(new BalanceCommand(
                        () -> m_NavX.getPitch(),
                        () -> m_NavX.getRoll(),
                        m_drivetrain));
            }
        }

        if (useStatusLights) {
            new Trigger(() -> m_controller.getRawButton(9)).whileTrue(new InstantCommand(() -> m_neoPixel.setAll(m_neoPixel.fullColor)));
        }

        //BUTTONBOARD TRIGGERS
        if (useButtonBoard) {
            m_buttonboardconfig = new ButtonBoardConfig();
        }

//        new Trigger(m_buttonboard1::leftSub).onTrue();
//        new Trigger(m_buttonboard1::rightSub).onTrue();
//        new Trigger(m_buttonboard1::floor).onTrue();
//        new Trigger(m_buttonboard1::store).onTrue();
//        new Trigger(m_buttonboard1::grid1).onTrue();
//        new Trigger(m_buttonboard1::grid2).onTrue();
//        new Trigger(m_buttonboard1::grid3).onTrue();
//        new Trigger(m_buttonboard1::grid4).onTrue();
//        new Trigger(m_buttonboard1::grid5).onTrue();
//        new Trigger(m_buttonboard1::grid6).onTrue();
//        new Trigger(m_buttonboard1::grid7).onTrue();
//        new Trigger(m_buttonboard2::grid8).onTrue();
//        new Trigger(m_buttonboard2::grid9).onTrue();
//        new Trigger(m_buttonboard2::confirm).onTrue();
//        new Trigger(m_buttonboard2::cancel).onTrue();


        if (useDrive && driveType.equals("SwerveDrive")) {
            new Trigger(() -> m_controller.getRawButton(7)).whileTrue(new BalanceCommand(
                    () -> m_NavX.getPitch(),
                    () -> m_NavX.getRoll(),
                    m_drivetrain));

        }

//        if(useDrive &&driveType.equals("SwerveDrive")) {
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
//
//
//            var commandPlayer = Shuffleboard.getTab("Path Following Commands");
//
//            var PPSwerveCommandPlayer = commandPlayer.
//                    getLayout("PP Swerve Commands", BuiltInLayouts.kGrid)
//                    .withPosition(0, 0)
//                    .withSize(4, 4);
//            PPSwerveCommandPlayer.add("Straight Path",
//                    getPathFollowCommand("Straight Path", Paths.straightPath));
//            PPSwerveCommandPlayer.add("180 while going forward path",
//                    getPathFollowCommand("Straight Path With 180", Paths.straight180Path));
//            PPSwerveCommandPlayer.add("Circular path",
//                    getPathFollowCommand("Circular Path", Paths.circularPath));
//            PPSwerveCommandPlayer.add("Auto1 Path (have more space!)",
//                    getPathFollowCommand("Auto1 Path", Paths.Auto1));
//            PPSwerveCommandPlayer.add("Test Path (have space!)",
//                    getPathFollowCommand("Test Path", Paths.testPath));
//            PPSwerveCommandPlayer.add("TPath",
//                    getPathFollowCommand("T Path", Paths.tPath));
//            PPSwerveCommandPlayer.add("Diagonal Path",
//                    getPathFollowCommand(Paths.diagonalPath));
//
//            var FollowPathCommandPlayer = commandPlayer.
//                    getLayout("Follow Path Commands", BuiltInLayouts.kGrid)
//                    .withPosition(4, 0)
//                    .withSize(4, 4);
//            FollowPathCommandPlayer.add("Straight our command",
//                    new FollowPathCommand(Paths.straightPath, m_drivetrain));
//            FollowPathCommandPlayer.add("180 while going forward our command",
//                    new FollowPathCommand(Paths.straight180Path, m_drivetrain));
//            FollowPathCommandPlayer.add("Circular our command",
//                    new FollowPathCommand(Paths.circularPath, m_drivetrain));
//            FollowPathCommandPlayer.add("Auto1 (have more space!) our command",
//                    new FollowPathCommand(Paths.Auto1, m_drivetrain));
//            FollowPathCommandPlayer.add("Test (have space!) our command",
//                    new FollowPathCommand(Paths.testPath, m_drivetrain));
//            FollowPathCommandPlayer.add("T our command",
//                    new FollowPathCommand(Paths.tPath, m_drivetrain));
//            FollowPathCommandPlayer.add("Diagonal Path with our command",
//                    new FollowPathCommand(Paths.diagonalPath, m_drivetrain));
//
//            HashMap<String, Command> commandHashMap = new HashMap<>();
//            commandHashMap.put("halfway", new PrintCommand("Passed Halfway!"));
//            FollowPathWithEvents pathWithEvents = new FollowPathWithEvents(
//                    new FollowPathCommand(Paths.straightPath, m_drivetrain),
//                    Paths.straightPath.getMarkers(),
//                    commandHashMap
//            );
//            commandPlayer.add("Straight Path With Events", pathWithEvents).withPosition(5, 0);

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


//        m_lidarIndicatorCommand = new LidarIndicatorCommand(m_stormNet);
//        m_lidarIndicatorCommand.schedule();

        if (useDrive) {
            m_drivetrain.onEnable();
            if (driveType.equals("SwerveDrive")) {
                m_poseEstimator.onEnable();
            }
        }
        System.out.println("-------------enabled-------------");
    }

    void onDisable() {
        m_robotState.onDisable();
        if (useDrive) {
            m_drivetrain.onDisable();
            if (driveType.equals("SwerveDrive")) {
                m_poseEstimator.onDisable();
            }
        }
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

