// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.auto.AutoCommand;
import frc.robot.commands.auto.AutoRoutines;
import frc.robot.commands.autoScoring.NodeSelector;
import frc.robot.commands.drive.BalanceCommand;
import frc.robot.commands.autoScoring.AutoScore;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.LidarIndicatorCommand;
import frc.robot.commands.arm.ArmCommand;
import frc.robot.commands.arm.BasicArm;
import frc.robot.commands.drive.EnhancedDriveWithJoystick;
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
    NodeSelector nodeSelector;
    StormXboxController xboxController;
    ButtonBoard buttonBoard;
    ButtonBoardConfig buttonBoardConfig;

    private final SendableChooser<AutoCommand> autoCommandChooser = new SendableChooser<>();


    public RobotContainer() throws IllegalDriveTypeException {
        //init constants
        FieldConstants.init();
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
//                System.out.println("Successfully created Drivetrain!");
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
            m_poseEstimator = new PoseEstimator(m_drivetrain.getSwerveDriveKinematics());
            System.out.println("USING pose estimator");
            Toggles.usePoseEstimator = true;
        }


        if (Toggles.useStormNet) {
            System.out.println("Using StormNet");
            StormNet.init();
            m_stormNet = StormNet.getInstance();
            m_stormNet.test();
//            var tab = Shuffleboard.getTab("Storm Net");
//            tab.addNumber("Lidar Distance", m_stormNet::getLidarDistance);
        } else {
            System.out.println("NOT using StormNet");
        }

        if (Toggles.useNodeSelector) {
            nodeSelector = new NodeSelector();
        } else {
            System.out.println("NOT using node selector");
        }

        // create controllers
        if (Toggles.useLogitechController) {
            logitechController = new StormLogitechController(kLogitechControllerPort);
            System.out.println("using logitech controller");
        } else {
            System.out.println("NOT using logitech controller");
        }

        if (Toggles.useXboxController) {
            xboxController = new StormXboxController(kLogitechControllerPort + 1);
            System.out.println("using xbox controller");
        } else
            System.out.println("NOT using xbox controller");

        // Configure the trigger bindings
        configureBindings();
    }

    private void configureBindings() {
        if (Toggles.useXboxController) {
            if (Toggles.useArm) {
                if (Toggles.useXYArmMode) {
                    System.out.println("Using XY mode for arm movement");
//                    m_armCommand = new XYArm(m_arm,
//                            xboxController::getRightJoystickX,
//                            xboxController::getLeftJoystickY);
                    m_armCommand = new XYArm(m_arm,
                            xboxController::getRightJoystickX,
                            xboxController::getRightJoystickY);
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
//                new Trigger(xboxController::getAButtonIsHeld).onTrue(new InstantCommand(m_compression::stopCompressor));
//                new Trigger(xboxController::getYButtonIsHeld).onTrue(new InstantCommand(m_compression::startCompressor));
            } else {
                System.out.println("Pneumatics or controller not operational");
            }

            if (Toggles.useNodeSelector) {
                new Trigger(xboxController::getUpArrowPressed)
                        .onTrue(new InstantCommand(() -> nodeSelector.moveSelectedRow(-1)));
                new Trigger(xboxController::getDownArrowPressed)
                        .onTrue(new InstantCommand(() -> nodeSelector.moveSelectedRow(1)));
                new Trigger(xboxController::getLeftArrowPressed)
                        .onTrue(new InstantCommand(() -> nodeSelector.moveSelectedCol(-1)));
                new Trigger(xboxController::getRightArrowPressed)
                        .onTrue(new InstantCommand(() -> nodeSelector.moveSelectedCol(1)));
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
            new Trigger(() -> logitechController.getWPIPOVAngle() != -1).onTrue(
                    new InstantCommand(
                        () -> {
                            double angle = logitechController.getWPIPOVAngle();
                            System.out.println("Set point angle: " + angle);
                            driveWithJoystick.setSetPoint(angle);
                        })
            );

            m_gyrocommand = new GyroCommand(m_drivetrain, 180);
            buttonBoard = new ButtonBoard(0);

            // zero angle command when we are red make sure robot pointing forwards is 180
//            new Trigger(() -> m_controller.getRawButton(4)).whileTrue(new GyroCommand(m_drivetrain, 180));
            new Trigger(() -> logitechController.getRawButton(5)).onTrue(
                    new InstantCommand(() -> {
                        m_drivetrain.getCurrentCommand().cancel();
                        driveWithJoystick.schedule();
                    })
            );
        }

        //BUTTONBOARD TRIGGERS
        if (Toggles.useButtonBoard) {
            buttonBoardConfig = new ButtonBoardConfig(m_neoPixel);
            buttonBoardConfig.buttonBoardSetup();
        }

        if (Toggles.useLogitechController && Toggles.useNavX) {
            new Trigger(() -> logitechController.getRawButton(8)).onTrue(new InstantCommand(() -> {
                double angle = (DriverStation.getAlliance() == DriverStation.Alliance.Red) ?
                        180.0 : 0;
                m_navX.setAngle(angle);
                if (Toggles.usePoseEstimator) {
                    m_poseEstimator.resetEstimator(new Pose2d(
                            RobotState.getInstance().getCurrentPose().getTranslation(),
                            m_navX.getAbsoluteRotation()));
                }
            }));
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

        if (Toggles.useNodeSelector && Toggles.useXboxController && Toggles.usePoseEstimator) {
            new Trigger(xboxController::getAButtonIsHeld).onTrue(
                    new AutoScore(m_drivetrain, nodeSelector::getSelectedNode)
            );
        }


        if (Toggles.usePoseEstimator) {
            if (AutoRoutines.autoCommands.size() > 0) {
                for (var command : AutoRoutines.autoCommands) {
                    autoCommandChooser.addOption(command.name, command);
                }
                var firstCommand = AutoRoutines.autoCommands.get(0);
                autoCommandChooser.setDefaultOption(firstCommand.name, firstCommand);
            }

            ShuffleboardConstants.getInstance().driverTab
                    .add("Auto", autoCommandChooser)
                    .withWidget(BuiltInWidgets.kComboBoxChooser)
                    .withPosition(3, 3).withSize(2, 1);
        }

//
//            SendableChooser<PathPlannerTrajectory> testPathChooser = new SendableChooser<>();
//            for (var path : Paths.listOfPaths) {
//                testPathChooser.addOption(path.name, path.path);
//            }
//            testPathChooser.setDefaultOption(Paths.listOfPaths.get(0).name, Paths.listOfPaths.get(0).path);
//
//            ShuffleboardConstants.getInstance().pathFollowingTab
//                    .add("Path Command", testPathChooser)
//                    .withPosition(0, 3).withSize(2, 1);
//
//            ShuffleboardConstants.getInstance().pathFollowingTab
//                    .add("Run Selected Command",
//                            new FolllowPathFromPose(m_drivetrain, testPathChooser.getSelected()))
//                    .withPosition(0, 2).withSize(2, 1)
//                    .withWidget(BuiltInWidgets.kCommand);

    }

    public Command getAutonomousCommand() {
        if (Toggles.usePoseEstimator) {
            AutoCommand selected = autoCommandChooser.getSelected();
            System.out.println("Auto: " + selected.name);
            m_robotState.setStartPose(selected.startPose);
            return selected.autoCommand;
        }
        return new PrintCommand("Autonomous! -----");
    }
}

