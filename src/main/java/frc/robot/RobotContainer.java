// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.autoScoring.NodeSelector;
import frc.robot.commands.drive.BalanceCommand;
import frc.robot.commands.autoScoring.AutoScore;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.LidarIndicatorCommand;
import frc.robot.commands.arm.ArmCommand;
import frc.robot.commands.arm.BasicArm;
import frc.robot.commands.drive.EnhancedDriveWithJoystick;
import frc.robot.commands.drive.pathFollowing.FolllowPathFromPose;
import frc.robot.commands.drive.pathFollowing.PathFollowingCommand;
import frc.robot.commands.drive.pathFollowing.Paths;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ShuffleboardConstants;
import frc.robot.commands.arm.XYArm;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.NeoPixel;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.stormnet.StormNet;
import frc.robot.subsystems.Compression;
import frc.robot.commands.drive.GyroCommand;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.subsystems.drive.DrivetrainFactory;

import frc.robot.subsystems.drive.IllegalDriveTypeException;
import frc.robot.subsystems.vision.Vision;
import frc.utils.joysticks.ButtonBoard;
import frc.utils.joysticks.ButtonBoardConfig;
import frc.utils.joysticks.StormLogitechController;
import frc.utils.joysticks.StormXboxController;

import java.util.HashMap;
import java.util.Map;

import static frc.robot.constants.Constants.*;


public class RobotContainer {

    // **********
    // SUBSYSTEMS
    // **********
    DrivetrainBase m_drivetrain;
    Compression m_compression;
    Arm m_arm;
    StormNet m_stormNet;
    NavX m_navX;
    Vision m_vision;
    PoseEstimator m_poseEstimator;
    NodeSelector m_nodeSelector;
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
    final RobotState m_robotState;


    // **********
    // Controllers
    // **********
    StormLogitechController logitechController;
    StormXboxController xboxController;
    ButtonBoard buttonBoard;
    ButtonBoardConfig buttonBoardConfig;

    private final SendableChooser<Paths.PathWithName> autoPathChooser = new SendableChooser<>();
    private final Map<String, Command> autoEventMap = new HashMap<>();


    public RobotContainer() throws IllegalDriveTypeException {
        //init constants
        FieldConstants.Grids.initGridNodes();
        ShuffleboardConstants.getInstance();

        m_robotState = RobotState.getInstance();

        if (Toggles.useNavX) {
            m_navX = new NavX();
        } else {
            System.out.println("NOT using navX");
        }

        if (Toggles.useStatusLights) {
            m_neoPixel = new NeoPixel();
            System.out.println("Using StatusLights");
        } else {
            System.out.println("NOT using StatusLights");
        }

        // Note the pattern of attempting to create the object then disabling it if that creation fails
        if (Toggles.useDrive) {
            try {
                m_drivetrain = DrivetrainFactory.getInstance(driveType);
                System.out.println("Successfully created Drivetrain!");
            } catch (Exception e) {
                e.printStackTrace();
                 Toggles.useDrive = false;
                System.out.println("NOT using drive - caught exception!");
            }
        } else {
            System.out.println("NOT using drive");
        }

        if (Toggles.useVision) {
            m_vision = new Vision();
        } else {
            System.out.println("NOT using vision");
        }

        if (Toggles.useArm) {
            m_arm = new Arm();
            System.out.println("Using arm");
        } else {
            System.out.println("NOT using arm");
        }

        if (Toggles.usePneumatics) {
            m_compression = new Compression();
        } else
            System.out.println("NOT using pneumatics");

        if (Toggles.useStormNet) {
          StormNet.init();
          m_stormNet = StormNet.getInstance();
        } else
            System.out.println("NOT using stormnet");

        if (Toggles.useDrive && driveType.equals("SwerveDrive")) {
            m_poseEstimator = new PoseEstimator(
                    m_drivetrain.getSwerveDriveKinematics(),
                    m_drivetrain.getSwerveModulePositions()
            );
            System.out.println("USING pose estimator");
            Toggles.usePoseEstimator = true;
        }

        if (Toggles.useLogitechController) {
            logitechController = new StormLogitechController(kLogitechControllerPort);
            xboxController = new StormXboxController(kLogitechControllerPort + 1);
            System.out.println("using logitech controller");
        } else {
            System.out.println("NOT using logitech controller");
        }

        if (Toggles.useStormNet) {
            System.out.println("Using StormNet");
            StormNet.init();
            m_stormNet = StormNet.getInstance();
            m_stormNet.test();
            var tab = Shuffleboard.getTab("Storm Net");
            tab.addNumber("Lidar Distance", m_stormNet::getLidarDistance);
        } else {
            System.out.println("NOT using StormNet");
        }

        if (Toggles.useNodeSelector) {
            m_nodeSelector = new NodeSelector();
        } else {
            System.out.println("NOT using node selector");
        }

        // Configure the trigger bindings
        configureBindings();
    }

    private void configureBindings() {
        if (Toggles.useXboxController) {
            xboxController = new StormXboxController(1);
            if (Toggles.useArm) {
                if (Toggles.useXYArmMode) {
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

            if (Toggles.usePneumatics) {
                new Trigger(xboxController::getXButtonIsHeld).onTrue(new InstantCommand(m_compression::grabCubeOrCone));
                new Trigger(xboxController::getBButtonIsHeld).onTrue(new InstantCommand(m_compression::release));
                new Trigger(xboxController::getAButtonIsHeld).onTrue(new InstantCommand(m_compression::stopCompressor));
                new Trigger(xboxController::getYButtonIsHeld).onTrue(new InstantCommand(m_compression::startCompressor));
            } else {
                System.out.println("Pneumatics or controller not operational");
            }

            if (Toggles.useNodeSelector) {
                new Trigger(xboxController::getUpArrowPressed)
                        .onTrue(new InstantCommand(() -> m_nodeSelector.moveSelectedRow(-1)));
                new Trigger(xboxController::getDownArrowPressed)
                        .onTrue(new InstantCommand(() -> m_nodeSelector.moveSelectedRow(1)));
                new Trigger(xboxController::getLeftArrowPressed)
                        .onTrue(new InstantCommand(() -> m_nodeSelector.moveSelectedCol(-1)));
                new Trigger(xboxController::getRightArrowPressed)
                        .onTrue(new InstantCommand(() -> m_nodeSelector.moveSelectedCol(1)));
                System.out.println("using controller to control node selector");
            }
        }

        if (Toggles.useDrive && Toggles.useLogitechController) {
            EnhancedDriveWithJoystick driveWithJoystick = new EnhancedDriveWithJoystick(
                    m_drivetrain,
                    logitechController::getWpiXAxis,
                    logitechController::getWpiYAxis,
                    logitechController::getWpiZAxis,
                    () -> logitechController.getRawButton(1),
                    () -> logitechController.getRawButton(2)
            );
            m_drivetrain.setDefaultCommand(driveWithJoystick);
            // set setpoints using pov angle
            new Trigger(() -> logitechController.getWPIPOVAngle() != -1).onTrue(new InstantCommand(
                    () -> {
                        double angle = logitechController.getWPIPOVAngle();
                        System.out.println("Set point angle: " + angle);
                        driveWithJoystick.setSetPoint(angle);
                    }));

            m_gyrocommand = new GyroCommand(m_drivetrain, 180);
            buttonBoard = new ButtonBoard(0);

            // zero angle command when we are red make sure robot pointing forwards is 180
//            new Trigger(() -> m_controller.getRawButton(4)).whileTrue(new GyroCommand(m_drivetrain, 180));
            new Trigger(() -> logitechController.getRawButton(5)).onTrue(new InstantCommand(() -> {
                m_drivetrain.getCurrentCommand().cancel();
                driveWithJoystick.schedule();
            }));
        }

        if (Toggles.useLogitechController && Toggles.useStatusLights) {
            new Trigger(() -> logitechController.getRawButton(9)).whileTrue(new InstantCommand(() -> m_neoPixel.setAll(NeoPixel.fullColor)));
        }

        //BUTTONBOARD TRIGGERS
        if (Toggles.useButtonBoard) {
            buttonBoardConfig = new ButtonBoardConfig();
        }

        if (Toggles.useLogitechController && Toggles.useNavX) {
            new Trigger(() -> logitechController.getRawButton(8)).onTrue(new InstantCommand(() -> {
                double angle = (DriverStation.getAlliance() == DriverStation.Alliance.Red) ?
                        180.0 : 0;
                m_navX.setAngle(angle);
                if (Toggles.usePoseEstimator) {
                    m_poseEstimator.resetEstimator(m_navX.getAbsoluteRotation(), m_drivetrain.getSwerveModulePositions(),
                            new Pose2d(RobotState.getInstance().getCurrentPose().getTranslation(),
                                    Rotation2d.fromDegrees(angle)));
                }
            }));
        }

        if (Toggles.useDrive && driveType.equals("SwerveDrive") && Toggles.usePoseEstimator) {

            autoEventMap.put("PickUpFromGround", new PrintCommand("Picking up game piece from ground!"));
            autoEventMap.put("StowArm", new PrintCommand("Stowing the arm!"));
            autoEventMap.put("LiftArm", new PrintCommand("Lifting the arm!"));
            autoEventMap.put("PlaceGamePiece", new PrintCommand("Placing the game piece!"));
            autoEventMap.put("Balance", new PrintCommand("Balancing on charging station!"));

            //add paths to chooser
            for (var pathWithName : Paths.listOfPaths) {
                autoPathChooser.addOption("Auto " + pathWithName.name, pathWithName);
            }
            autoPathChooser.setDefaultOption(Paths.listOfPaths.get(0).name, Paths.listOfPaths.get(0));
            SmartDashboard.putData("Auto Paths", autoPathChooser);

            SendableChooser<PathPlannerTrajectory> testPathChooser = new SendableChooser<>();
            for (var path : Paths.listOfPaths) {
                testPathChooser.addOption(path.name, path.path);
            }
            testPathChooser.setDefaultOption(Paths.listOfPaths.get(0).name, Paths.listOfPaths.get(0).path);

            ShuffleboardConstants.getInstance().pathFollowingTab
                    .add("Path Command", testPathChooser)
                    .withPosition(0, 3).withSize(2, 1);

            ShuffleboardConstants.getInstance().pathFollowingTab
                    .add("Run Selected Command",
                            new FolllowPathFromPose(m_drivetrain, testPathChooser.getSelected()))
                    .withPosition(0, 2).withSize(2, 1)
                    .withWidget(BuiltInWidgets.kCommand);

        }

        if (Toggles.useNodeSelector && Toggles.useXboxController && Toggles.usePoseEstimator) {
            new Trigger(xboxController::getAButtonIsHeld)
                    .onTrue(new AutoScore(m_drivetrain, m_nodeSelector.getSelectedNode()));
        }
    }

    private Command getPathFollowCommand(PathPlannerTrajectory path) {
        return new PathFollowingCommand(m_drivetrain).withPath(path);
    }

    public Command getAutonomousCommand() {
        if (Toggles.useDrive && driveType.equals("SwerveDrive")) {
            var selectedPath = autoPathChooser.getSelected();
            m_robotState.setStartPose(selectedPath.path.getInitialHolonomicPose());
            return new FollowPathWithEvents(
                    new PathFollowingCommand(m_drivetrain, selectedPath.path, true),
                    selectedPath.path.getMarkers(),
                    autoEventMap
            );
        }
        return new PrintCommand("Autonomous! -----");
    }
}

