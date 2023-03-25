// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;

import frc.robot.commands.AprilTagStatusCommand;
import frc.robot.commands.arm.pathFollowing.ArmToTranslation;
import frc.robot.commands.auto.AutoRoutine;
import frc.robot.commands.auto.AutoRoutines;
import frc.robot.commands.auto.autoManeuvers.*;
import frc.robot.commands.drive.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.LEDcommand;
import frc.robot.commands.arm.ArmCommand;
import frc.robot.commands.arm.BasicArm;
import frc.robot.commands.drive.BalancePitchCommand;
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
import frc.robot.subsystems.vision.AprilTagStatusCommand;
import frc.robot.subsystems.vision.Vision;
import frc.utils.joysticks.ButtonBoard;
import frc.utils.joysticks.ButtonBoardConfig;
import frc.utils.joysticks.StormLogitechController;
import frc.utils.joysticks.StormXboxController;

import java.util.function.BooleanSupplier;

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
    LEDcommand m_LEDcommand;
    AprilTagStatusCommand m_aprilTagStatusCommand;
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
    StormXboxController firstXboxController;
    StormXboxController secondXboxController;
    ButtonBoard buttonBoard;
    ButtonBoardConfig buttonBoardConfig;

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
            m_aprilTagStatusCommand = new AprilTagStatusCommand(m_neoPixel,m_vision);
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


        // create controllers
        if (Toggles.useButtonBoard){
            buttonBoardConfig = new ButtonBoardConfig(m_neoPixel, nodeSelector, m_compression, m_arm);
            buttonBoardConfig.buttonBoardSetup();
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

        // Configure the trigger bindings
        configureBindings();
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
    }

    private void configureBindings() {
        if (Toggles.useFirstXboxController) {
            if (Toggles.useDrive) {
                EnhancedDriveWithJoystick driveWithJoystick = new EnhancedDriveWithJoystick(
                        m_drivetrain,
                        firstXboxController::getLeftJoystickY,
                        () -> -firstXboxController.getLeftJoystickX(),
                        () -> -firstXboxController.getRightJoystickX(),
                        firstXboxController::getRightBumperIsHeld,
                        () -> firstXboxController.getRightTrigger() > 0.2
                );
                m_drivetrain.setDefaultCommand(driveWithJoystick);

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
                            new ArmToTranslation(m_arm, ArmConstants.stowPosition, 2, 2));
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
                    () -> logitechController.getRawButton(1),
                    () -> logitechController.getRawButton(2)
            );
            m_drivetrain.setDefaultCommand(driveWithJoystick);
            // set setpoints using pov angle
//            new Trigger(() -> logitechController.getWPIPOVAngle() != -1).onTrue(
//                    new InstantCommand(
//                        () -> {
//                            double angle = logitechController.getWPIPOVAngle();
//                            System.out.println("Set point angle: " + angle);
//                            driveWithJoystick.setSetPoint(angle);
//                        })
//            );

            // 10 SHOULD BE FORWARD AND 12 SHOULD BE BACKWARD]
            new Trigger(() -> logitechController.getRawButton(10)).onTrue(new InstantCommand(
                    () -> driveWithJoystick.setSetPoint(0)
            ));
            new Trigger(() -> logitechController.getRawButton(12)).onTrue(new InstantCommand(
                    () -> driveWithJoystick.setSetPoint(180)
            ));

            if (Toggles.usePoseEstimator) {
                new Trigger(() -> logitechController.getRawButton(3)).whileTrue(
                        new AlignToDoubleSubstation(m_drivetrain,
                                logitechController::getWpiXAxis, logitechController::getWpiZAxis,
                                AlignToDoubleSubstation.Side.LEFT));
                new Trigger(() -> logitechController.getRawButton(4)).whileTrue(
                        new AlignToDoubleSubstation(m_drivetrain,
                                logitechController::getWpiXAxis, logitechController::getWpiZAxis,
                                AlignToDoubleSubstation.Side.RIGHT));
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

//        new Trigger(() -> logitechController.getRawButton(12)).onTrue(new InstantCommand(() -> {m_vision.setMode(0);
//            System.out.println("12 ran");}));
//        new Trigger(() -> logitechController.getRawButton(11)).onTrue(new InstantCommand(() -> {m_vision.setMode(1);
//            System.out.println("11 Ran");}));

        //BUTTONBOARD TRIGGERS
        if (Toggles.useButtonBoard) {

            if (Toggles.useStatusLights) {
                m_LEDcommand = new LEDcommand(m_stormNet, m_neoPixel, buttonBoardConfig);
                m_neoPixel.setDefaultCommand(m_LEDcommand);
            }

            // Button board can only do XY arm mode
            if (Toggles.useArm) {
                m_armCommand = new XYArm(m_arm,
                        buttonBoardConfig::armInOut,
                        buttonBoardConfig::armUpDown);
                m_arm.setDefaultCommand(m_armCommand);

                new Trigger(buttonBoardConfig::stow).onTrue(
                        new ArmToTranslation(m_arm, ArmConstants.stowPosition, 2, 2));
                new Trigger(buttonBoardConfig::pickFloor).onTrue(
                        new ArmToTranslation(m_arm, ArmConstants.pickGround, 2, 2));



            }
            if (Toggles.useStormNet && Toggles.useDrive && Toggles.usePneumatics && Toggles.useArm &&  Toggles.useNodeSelector) {
                new Trigger(() -> buttonBoardConfig.confirm() && !buttonBoardConfig.m_Position.equals(DriveToDoubleSubstation.Position.NONE)).onTrue
                        (new PickFromSubstationSequence(m_drivetrain, m_arm, m_compression, buttonBoardConfig.m_Position, m_stormNet));

                new Trigger(() -> buttonBoardConfig.confirm() && buttonBoardConfig.m_Position.equals(DriveToDoubleSubstation.Position.NONE)).onTrue(
                    new DropPieceSequence(m_drivetrain, m_arm, m_compression, nodeSelector));
            }

            new Trigger(buttonBoardConfig::cancel).onTrue(new InstantCommand(() ->
                    CommandScheduler.getInstance().cancelAll()
            ));

        }

        if (Toggles.useLogitechController && Toggles.useNavX) {
            BooleanSupplier isRed = () -> m_robotState.getCurrentAlliance() == DriverStation.Alliance.Red;
            new Trigger(() -> logitechController.getRawButton(8)).onTrue(new InstantCommand(() -> {
                double angle = isRed.getAsBoolean()?
                        180.0 : 0;
                m_navX.setAngle(angle);
                if (Toggles.usePoseEstimator) {
                    m_poseEstimator.resetEstimator(
                            new Pose2d(
                                RobotState.getInstance().getCurrentPose().getTranslation(),
                                m_navX.getAbsoluteRotation()));
                }
            }));

//            if (Toggles.useDrive && Toggles.usePoseEstimator) {
//                ShuffleboardConstants.getInstance().driverTab
//                        .add("Balance Pitch*", new BalancePitchCommand(m_drivetrain, m_navX::getPitch));
//                ShuffleboardConstants.getInstance().driverTab
//                        .add("Balance", new BalanceCommand(m_navX::getPitch, m_navX::getRoll, m_drivetrain));
//            }
        }
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

    public void enabledInit() {
//        m_robotState.setCurrentAlliance(allianceChooser.getSelected());
    }
}