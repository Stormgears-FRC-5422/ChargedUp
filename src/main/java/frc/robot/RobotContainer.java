// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.*;

import frc.robot.commands.AprilTagStatusCommand;
import frc.robot.commands.arm.pathFollowing.ArmToTranslation;
import frc.robot.commands.arm.pathFollowing.StowArm;
import frc.robot.commands.auto.AutoRoutine;
import frc.robot.commands.auto.AutoRoutines;
import frc.robot.commands.auto.autoManeuvers.*;
import frc.robot.commands.drive.*;
import frc.robot.commands.drive.BalanceCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.LEDcommand;
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
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.subsystems.drive.DrivetrainFactory;

import frc.robot.subsystems.drive.IllegalDriveTypeException;
import frc.robot.subsystems.vision.Vision;
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

    int[] allRingSegments = {1, 2, 3, 4};

    // **********
    // COMMANDS
    // **********
    BalanceCommand m_balancecommand;
    GyroCommand m_gyrocommand;
    LEDcommand m_LEDcommand;
    AprilTagStatusCommand m_aprilTagStatusCommand;
    ArmCommand m_armCommand;
    //    TrapezoidMoveForward trapezoidMoveForwardCommand = new TrapezoidMoveForward(m_drivetrain, 20, 1, 0.2);

    // **********
    // Other
    // **********
    final RobotState m_robotState;
    public FieldConstants.Side m_side = FieldConstants.Side.NONE;


    // **********
    // Controllers
    // **********
    StormLogitechController logitechController;
    NodeSelector nodeSelector;
    StormXboxController firstXboxController;
    StormXboxController secondXboxController;
    ButtonBoardConfig m_buttonBoardConfig;

    private final SendableChooser<AutoRoutine> autoCommandChooser = new SendableChooser<>();
    private final SendableChooser<DriverStation.Alliance> allianceChooser = new SendableChooser<>();

    public RobotContainer() throws IllegalDriveTypeException {
        //init constants
        FieldConstants.init();
        ShuffleboardConstants.getInstance();
//        allianceChooser.addOption("Blue", DriverStation.Alliance.Blue);
//        allianceChooser.addOption("Red", DriverStation.Alliance.Red);
//        allianceChooser.setDefaultOption("Blue", DriverStation.Alliance.Blue);
//        ShuffleboardConstants.getInstance().driverTab
//                .add("Alliance?", allianceChooser)
//                .withWidget(BuiltInWidgets.kComboBoxChooser)
//                .withPosition(4, 4).withSize(2, 1);

        m_robotState = RobotState.getInstance();
//        m_robotState.setCurrentAlliance(DriverStation.Alliance.Red);

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
                m_drivetrain = DrivetrainFactory.getInstance(DriveConstants.driveType);
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
            m_aprilTagStatusCommand = new AprilTagStatusCommand(m_neoPixel, m_vision);
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
            m_compression.startCompressor();
            System.out.println("Using Compressor");
        } else
            System.out.println("NOT using pneumatics");

        if (Toggles.useDrive && DriveConstants.driveType.equals("SwerveDrive")) {
            m_poseEstimator = new PoseEstimator(m_drivetrain.getSwerveDriveKinematics());
            System.out.println("USING pose estimator");
            Toggles.usePoseEstimator = true;
        }

        if (Toggles.useNodeSelector) {
            nodeSelector = new NodeSelector();
        } else {
            System.out.println("NOT using node selector");
        }

        if (Toggles.useStormNet) {
            System.out.println("Using StormNet");
            StormNet.init();
            m_stormNet = StormNet.getInstance();
            m_stormNet.test();
        } else {
            System.out.println("NOT using StormNet");
        }

        if (Toggles.useLogitechController) {
            logitechController = new StormLogitechController(kLogitechControllerPort);
            System.out.println("using logitech controller");
        } else {
            System.out.println("NOT using logitech controller");
        }

        if (Toggles.useFirstXboxController) {
            firstXboxController = new StormXboxController(kLogitechControllerPort + 4);
            System.out.println("using first xbox controller");
        } else
            System.out.println("NOT using first xbox controller");

        if (Toggles.useSecondXboxController) {
            secondXboxController = new StormXboxController(kLogitechControllerPort + 3);
            System.out.println("using second xbox controller");
        } else
            System.out.println("NOT using second xbox controller");

        if (Toggles.usePoseEstimator && Toggles.useNavX && Toggles.useArm &&
                Toggles.usePneumatics) {
//            System.out.println();
            AutoRoutines.initAutoRoutines(m_drivetrain, m_navX, m_arm, m_compression);

            if (AutoRoutines.autoRoutines.size() > 0) {
                for (var autoCommand : AutoRoutines.autoRoutines) {
                    autoCommandChooser.addOption(autoCommand.name, autoCommand);
                }
                var command = AutoRoutines.autoRoutines.get(0);
                autoCommandChooser.setDefaultOption(command.name, command);
            }

            ShuffleboardConstants.getInstance().driverTab
                    .add("Auto Selector", autoCommandChooser)
                    .withWidget(BuiltInWidgets.kComboBoxChooser)
                    .withPosition(2, 3).withSize(2, 1);
        }


        if (Toggles.useButtonBoard) {
            m_buttonBoardConfig = new ButtonBoardConfig();
                m_neoPixel.setSpecificSegmentColor(allRingSegments,
                        m_buttonBoardConfig.cubeSelected() ? NeoPixel.PURPLE_COLOR : NeoPixel.YELLOW_COLOR);
                if (m_buttonBoardConfig.topGrid()) {
                    nodeSelector.setSelectedRow(2);
                } else if (m_buttonBoardConfig.middleGrid()) {
                    nodeSelector.setSelectedRow(1);
                } else if (m_buttonBoardConfig.bottomGrid()) {
                    nodeSelector.setSelectedRow(0);
                }
            System.out.println("using ButtonBoard");
        } else {
            System.out.println("NOT using ButtonBoard");
        }

        // Configure the controller bindings
        configureControllerBindings();
        if (Toggles.useButtonBoard)
            configureButtonBoardBindings();
        configureOtherCommands();
    }



    private void configureControllerBindings() {
        if (Toggles.useFirstXboxController) {
            if (Toggles.useDrive) {
                EnhancedDriveWithJoystick driveWithJoystick = new EnhancedDriveWithJoystick(
                        m_drivetrain,
                        firstXboxController::getWpiXSpeed,
                        firstXboxController::getWpiYSpeed,
                        firstXboxController::getOmegaSpeed,
                        () -> firstXboxController.getLeftTrigger() > 0.2,
                        () -> firstXboxController.getRightTrigger() > 0.2
                );
                m_drivetrain.setDefaultCommand(driveWithJoystick);

                new Trigger(firstXboxController::getStickRightButton).onTrue(zeroRotationCommand());

                new Trigger(firstXboxController::getYButtonIsHeld).onTrue(new InstantCommand(() -> {
                    driveWithJoystick.setSetPoint(0);
                }));
                new Trigger(firstXboxController::getAButtonIsHeld).onTrue(new InstantCommand(() -> {
                    driveWithJoystick.setSetPoint(180);
                }));
                new Trigger(firstXboxController::getXButtonIsHeld).onTrue(new InstantCommand(() -> {
                    driveWithJoystick.setSetPoint(90);
                }));
                new Trigger(firstXboxController::getBButtonIsHeld).onTrue(new InstantCommand(() -> {
                    driveWithJoystick.setSetPoint(-90);
                }));

//                new Trigger(firstXboxController::getLeftBumperIsHeld)
//                        .whileTrue(new AlignToDoubleSubstation(m_drivetrain, firstXboxController,
//                                AlignToDoubleSubstation.Side.LEFT));
//                new Trigger(firstXboxController::getRightBumperIsHeld)
//                        .whileTrue(new AlignToDoubleSubstation(m_drivetrain, firstXboxController,
//                                AlignToDoubleSubstation.Side.RIGHT));

                new Trigger(firstXboxController::getLeftBumperIsHeld).whileTrue(
                        new PickFromDoubleSubstation2(m_drivetrain, m_arm, m_compression, m_stormNet, m_neoPixel,
                                firstXboxController, FieldConstants.Side.LEFT));

                new Trigger(firstXboxController::getLeftLittleButtonIsHeld).onTrue(new InstantCommand(() -> {
                    m_drivetrain.getCurrentCommand().cancel();
                    driveWithJoystick.schedule();
                }));

                if (Toggles.useNodeSelector) {
                    new Trigger(firstXboxController::getUpArrowPressed)
                            .onTrue(new InstantCommand(() -> nodeSelector.moveSelectedRow(-1)));
                    new Trigger(firstXboxController::getDownArrowPressed)
                            .onTrue(new InstantCommand(() -> nodeSelector.moveSelectedRow(1)));
                    new Trigger(firstXboxController::getLeftArrowPressed)
                            .onTrue(new InstantCommand(() -> nodeSelector.moveSelectedCol(-1)));
                    new Trigger(firstXboxController::getRightArrowPressed)
                            .onTrue(new InstantCommand(() -> nodeSelector.moveSelectedCol(1)));
                    System.out.println("using controller to control node selector");

                    if (Toggles.usePoseEstimator) {
                        new Trigger(firstXboxController::getRightLittleButtonIsHeld).onTrue(
                                new DriveToNode(m_drivetrain, nodeSelector::getSelectedNode)
                        );
                    }
                }
            }
        }

        if (Toggles.useSecondXboxController) {
            if (Toggles.useArm) {
                if (Toggles.useXYArmMode) {
                    System.out.println("Using XY mode for arm movement");
//                    m_armCommand = new XYArm(m_arm,
//                            xboxController::getRightJoystickX,
//                            xboxController::getLeftJoystickY);
                    m_armCommand = new XYArm(m_arm,
                            secondXboxController::getRightJoystickX,
                            secondXboxController::getRightJoystickY);
                    new Trigger(secondXboxController::getXButtonIsHeld).onTrue(
                            new ArmToTranslation(m_arm, ArmConstants.pickGround, 2, 2));
                    new Trigger(secondXboxController::getBButtonIsHeld).onTrue(
                            new StowArm(m_arm));
                } else {
                    System.out.println("Using Angle mode for arm movement");
                    m_armCommand = new BasicArm(m_arm,
                            secondXboxController::getLeftJoystickY,
                            secondXboxController::getRightJoystickY);
                }
                m_arm.setDefaultCommand(m_armCommand);

                new Trigger(secondXboxController::getLeftLittleButtonIsHeld).onTrue(
                        Commands.runOnce(() -> {
                            m_arm.getCurrentCommand().cancel();
                            m_arm.getDefaultCommand().schedule();
                        })
                );
            }

            if (Toggles.usePneumatics) {
                new Trigger(secondXboxController::getRightBumperIsHeld)
                        .onTrue(new InstantCommand(m_compression::grabCubeOrCone));
                new Trigger(secondXboxController::getLeftBumperIsHeld)
                        .onTrue(new InstantCommand(m_compression::release));
            } else {
                System.out.println("Pneumatics or controller not operational");
            }

        }

        if (Toggles.useDrive && Toggles.useLogitechController) {
            EnhancedDriveWithJoystick driveWithJoystick = new EnhancedDriveWithJoystick(
                    m_drivetrain,
                    logitechController::getWpiXAxis,
                    logitechController::getWpiYAxis,
                    logitechController::getWpiZAxis,
                    () -> logitechController.getRawButton(11),
                    () -> logitechController.getRawButton(2)
            );
            m_drivetrain.setDefaultCommand(driveWithJoystick);

            // 10 SHOULD BE FORWARD AND 12 SHOULD BE BACKWARD
            new Trigger(() -> logitechController.getRawButton(10)).onTrue(new InstantCommand(
                    () -> driveWithJoystick.setSetPoint(0)
            ));
            new Trigger(() -> logitechController.getRawButton(12)).onTrue(new InstantCommand(
                    () -> driveWithJoystick.setSetPoint(180)
            ));

            if (Toggles.usePoseEstimator) {
                new Trigger(() -> logitechController.getRawButton(3)).whileTrue(
                        new PickFromDoubleSubstation2(
                                m_drivetrain, m_arm, m_compression, m_stormNet, m_neoPixel,
                                logitechController,
                                FieldConstants.Side.LEFT));
                new Trigger(() -> logitechController.getRawButton(4)).whileTrue(
                        new PickFromDoubleSubstation2(
                                m_drivetrain, m_arm, m_compression, m_stormNet, m_neoPixel,
                                logitechController,
                                FieldConstants.Side.RIGHT));

                new Trigger(() -> logitechController.getRawButton(3)).or(() -> logitechController.getRawButton(4))
                        .onFalse(new StowArm(m_arm)
                               );
            }

            if (Toggles.useNodeSelector) {
//                new Trigger(() -> logitechController.getRawButton(1)).whileTrue(
//                        new DriveToNode(m_drivetrain, nodeSelector::getSelectedNode)
//                );

                // TODO: test to see if this works aligns similar to pickup and placing is based on buttonboard
                if (Toggles.useArm && Toggles.usePneumatics && Toggles.useButtonBoard) {
                    new Trigger(() -> logitechController.getRawButton(1)).whileTrue(
                            new ComplexAutoScore(m_drivetrain, m_arm, m_compression,
                                    nodeSelector::getSelectedNode, logitechController, m_buttonBoardConfig::confirm)
                    );

//                    new Trigger(() -> logitechController.getRawButton(1))
//                            .onFalse(m_compression.getReleaseCommand().andThen(new StowArm(m_arm)));
                }

                // TODO: possible control for driver which automatically places the piece
//                if (Toggles.useArm && Toggles.usePneumatics) {
//                    new Trigger(() -> logitechController.getRawButton(1)).whileTrue(
//                            new AutoScore(m_drivetrain, m_arm, m_compression, nodeSelector::getSelectedNode)
//                    );
//                }
            }
//            m_gyrocommand = new GyroCommand(m_drivetrain, 180);
//            new Trigger(() -> m_controller.getRawButton(4)).whileTrue(new GyroCommand(m_drivetrain, 180));
            // zero angle command when we are red make sure robot pointing forwards is 180
            new Trigger(() -> logitechController.getRawButton(5)).onTrue(
                    new InstantCommand(() -> {
                        System.out.println("Cancelling current drivetrain command!");
                        m_drivetrain.getCurrentCommand().cancel();
                        driveWithJoystick.schedule();
                    })
            );
        }

        if (Toggles.useLogitechController && Toggles.useNavX) {
            new Trigger(() -> logitechController.getRawButton(8)).onTrue(zeroRotationCommand());
        }
    }

    private void configureButtonBoardBindings() {
        System.out.println("configButtonBoardBindings starting");

        // **********
        // Major safety triggers
        // **********
        new Trigger(m_buttonBoardConfig::kill).onTrue(new InstantCommand(() -> System.exit(0)));

        new Trigger(m_buttonBoardConfig::cancel).onTrue(new InstantCommand(() -> {
            CommandScheduler.getInstance().cancelAll();
            m_side = FieldConstants.Side.NONE;
        }));

        // **********
        // Switches for state - cone/cube, node levels
        // **********
        if (Toggles.useNodeSelector) {
            new Trigger(m_buttonBoardConfig::topGrid)
                    .whileTrue(new InstantCommand(() -> nodeSelector.setSelectedRow(2)));
            new Trigger(m_buttonBoardConfig::middleGrid)
                    .whileTrue(new InstantCommand(() -> nodeSelector.setSelectedRow(1)));
            new Trigger(m_buttonBoardConfig::bottomGrid)
                    .whileTrue(new InstantCommand(() -> nodeSelector.setSelectedRow(0)));
            // Set grid 1 - 9
            for (int i = 0; i <= 8; i++) {
                int tmpI = i; // Need a final value for the lambda function
                new Trigger(() -> m_buttonBoardConfig.getGridButton(tmpI))
                        .onTrue(new InstantCommand(() -> {
                            nodeSelector.setSelectedCol(tmpI);
                        }));
            }
        }

        new Trigger(m_buttonBoardConfig::cubeSelected).whileTrue(new InstantCommand(() -> {
            m_neoPixel.setSpecificSegmentColor(allRingSegments, NeoPixel.PURPLE_COLOR);
            RobotState.getInstance().setLidarRange(LidarRange.CUBE);
        }));
        new Trigger(m_buttonBoardConfig::cubeSelected).whileFalse(new InstantCommand(() -> {
            m_neoPixel.setSpecificSegmentColor(allRingSegments, NeoPixel.YELLOW_COLOR);
            RobotState.getInstance().setLidarRange(LidarRange.CONE);
        }));




        // **********
        // Gripper
        // **********
        new Trigger(m_buttonBoardConfig::gripperClosed)
                .whileTrue(m_compression.getGrabCommand());
        new Trigger(m_buttonBoardConfig::gripperClosed)
                .whileFalse(m_compression.getReleaseCommand());

        // **********
        // Automated routines for arm placement
        // **********
        if (Toggles.useArm) {
            // Stow, pick floor
            new Trigger(m_buttonBoardConfig::stow).onTrue(
                    new StowArm(m_arm));
            new Trigger(m_buttonBoardConfig::pickFloor).onTrue(
                    new ArmToTranslation(m_arm, ArmConstants.pickGround, 2, 2));


            if (Toggles.useStormNet && Toggles.useDrive && Toggles.usePneumatics && Toggles.useNodeSelector) {
//                new Trigger(() -> m_buttonBoardConfig.confirm() && m_side != FieldConstants.Side.NONE)
//                        .onTrue(new PickFromSubstationSequence(m_drivetrain, m_arm, m_compression,
//                                m_side, m_stormNet, logitechController));
//                new Trigger(() -> m_buttonBoardConfig.confirm() && m_side != FieldConstants.Side.NONE)
//                        .onTrue(new ArmToPickUp(m_arm, m_stormNet));

                new Trigger(() -> m_buttonBoardConfig.confirm() && m_side == FieldConstants.Side.NONE).onTrue(
                        new DropPieceSequence(m_drivetrain, m_arm, m_compression, nodeSelector));
            }

//            new Trigger(m_buttonBoardConfig::confirm).onTrue(
//                    new ArmToNode(m_arm, nodeSelector::getSelectedNode));
        }

        if (Toggles.useDrive && Toggles.useArm & Toggles.usePneumatics) {
//            new Trigger(m_buttonBoardConfig::pickLeftSub).onTrue(
//                    new ArmToTranslation(m_arm, ArmConstants.pickDoubleSubstation, 2, 2));
//            new Trigger(m_buttonBoardConfig::pickRightSub).onTrue(
//                    new ArmToTranslation(m_arm, ArmConstants.pickDoubleSubstation, 2, 2));
                    // set pickup substation info
                    new Trigger(m_buttonBoardConfig::pickLeftSub).onTrue(new InstantCommand(() -> {
                        m_side = FieldConstants.Side.LEFT;
                    }));
                    new Trigger(m_buttonBoardConfig::pickRightSub).onTrue(new InstantCommand(() -> {
                        m_side = FieldConstants.Side.RIGHT;
                    }));
        }

        //            if (Toggles.useNodeSelector && Toggles.usePoseEstimator &&
        //                    Toggles.useVision && Toggles.useXYArmMode) {
        //                new Trigger(buttonBoardConfig::).onTrue(
        //                        new DriveToNode(m_drivetrain, nodeSelector::getSelectedNode));
        //
        //            }
    }

    public Command getAutonomousCommand() {
        if (Toggles.usePoseEstimator) {
            AutoRoutine selected = autoCommandChooser.getSelected();
            System.out.println("Auto: " + selected.name);
            m_robotState.setStartPose(selected.startPose);
//            m_navX.setAngle(selected.startPose.getRotation().getDegrees());
//            m_robotState.setGyroData(Timer.getFPGATimestamp(), m_navX.getAbsoluteRotation());
//            m_drivetrain.updateOdometryData();
            return selected.autoCommand;
        }
        return new PrintCommand("Autonomous! -----");
    }

    private void configureOtherCommands() {
        if (Toggles.useArm && Toggles.usePneumatics) {
            ShuffleboardConstants.getInstance().driverTab
                    .add("Move to high node cone",
                            new SequentialCommandGroup(
                                    new ArmToNode(m_arm, () -> FieldConstants.Grids.getGrid()[0][0]),
                                    new InstantCommand(m_compression::release)));
        }

        if (Toggles.useButtonBoard && Toggles.useStatusLights && Toggles.useStormNet) {
            m_LEDcommand = new LEDcommand(m_stormNet, m_neoPixel, m_compression);
            m_neoPixel.setDefaultCommand(m_LEDcommand);
        }

        if (Toggles.useButtonBoard && Toggles.useArm) {
            m_armCommand = new XYArm(m_arm,
                    m_buttonBoardConfig::armInOut,
                    m_buttonBoardConfig::armUpDown);
            m_arm.setDefaultCommand(m_armCommand);
        }
    }

    public void enabledInit() {
//        m_robotState.setCurrentAlliance(allianceChooser.getSelected());
    }

    private Command zeroRotationCommand() {
        return new InstantCommand(() -> {
            double angle = m_robotState.getCurrentAlliance() == DriverStation.Alliance.Red ?
                    180.0 : 0;
            m_navX.setAngle(angle);
            if (Toggles.usePoseEstimator) {
                m_poseEstimator.resetEstimator(
                        new Pose2d(
                                RobotState.getInstance().getCurrentPose().getTranslation(),
                                m_navX.getAbsoluteRotation()));
            }
        });
    }
}