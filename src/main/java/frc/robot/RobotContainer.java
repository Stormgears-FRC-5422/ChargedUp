// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.drive.BalanceCommand;
import frc.robot.commands.autoScoring.DriveToScoringNode;
import frc.robot.commands.drive.DriveWithJoystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.arm.BasicArm;
import frc.robot.commands.drive.pathFollowing.FolllowPathFromPose;
import frc.robot.commands.drive.pathFollowing.PathFollowingCommand;
import frc.robot.commands.drive.pathFollowing.Paths;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ShuffleboardConstants;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.stormnet.StormNet;
import frc.robot.subsystems.Compression;
import frc.robot.commands.drive.GyroCommand;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.subsystems.drive.DrivetrainFactory;

import frc.robot.subsystems.drive.IllegalDriveTypeException;
import frc.robot.subsystems.vision.Vision;
import frc.utils.joysticks.ButtonBoard;
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


    // **********
    // COMMANDS
    // **********
    BalanceCommand m_balancecommand;
    GyroCommand m_gyrocommand;
    BasicArm m_basicArm;


    // **********
    // Other
    // **********
    final RobotState m_robotState;
    PoseEstimator m_poseEstimator;

    StormLogitechController m_controller;

    private final SendableChooser<Paths.PathWithName> autoPathChooser = new SendableChooser<>();
    private final Map<String, Command> autoEventMap = new HashMap<>();
    ButtonBoard m_buttonboard;


    public RobotContainer() throws IllegalDriveTypeException {
        //init constants
        FieldConstants.Grids.initGridNodes();
        ShuffleboardConstants.getInstance();

        m_robotState = RobotState.getInstance();

        if (SubsystemToggles.useNavX) {
            m_navX = new NavX();
        } else {
            System.out.println("NOT using navX");
        }

        // Note the pattern of attempting to create the object then disabling it if that creation fails
        if (SubsystemToggles.useDrive) {
            try {
                m_drivetrain = DrivetrainFactory.getInstance(driveType);
            } catch (Exception e) {
                e.printStackTrace();
                 SubsystemToggles.useDrive = false;
                System.out.println("NOT using drive - caught exception!");
            }
        } else {
            System.out.println("NOT using drive");
        }

        if (SubsystemToggles.useVision) {
            m_vision = new Vision();
        } else {
            System.out.println("NOT using vision");
        }

        if (SubsystemToggles.useArm) {
            m_arm = new Arm();
        } else {
            System.out.println("NOT using arm");
        }

        if (SubsystemToggles.usePneumatics) {
            m_compression = new Compression();
        } else
            System.out.println("NOT using pneumatics");

        if (SubsystemToggles.useStormNet) {
          StormNet.init();
          m_stormNet = StormNet.getInstance();
        } else
            System.out.println("NOT using stormnet");

        if (SubsystemToggles.useDrive && driveType.equals("SwerveDrive")) {
            m_poseEstimator = new PoseEstimator(
                    m_drivetrain.getSwerveDriveKinematics(),
                    m_drivetrain.getSwerveModulePositions()
            );
            System.out.println("USING pose estimator");
            SubsystemToggles.usePoseEstimator = true;
        }

        // TODO - how do we know that this worked? e.g. what fails if the joystick is unplugged?
        if (SubsystemToggles.useController) {
            m_controller = new StormLogitechController(kLogitechControllerPort);
            System.out.println("using controller");
        } else {
            System.out.println("NOT using controller");
        }

        if (SubsystemToggles.useStormNet) {
            System.out.println("Using StormNet");
            StormNet.init();
            m_stormNet = StormNet.getInstance();
            m_stormNet.test();
            var tab = Shuffleboard.getTab("Storm Net");
            tab.addNumber("Lidar Distance", m_stormNet::getLidarDistance);
        }

        // Configure the trigger bindings
        configureBindings();
    }

    private void configureBindings() {
        StormXboxController xboxController = new StormXboxController(1);

        if (SubsystemToggles.useArm && SubsystemToggles.useController) {
            m_basicArm = new BasicArm(m_arm,
                    xboxController::getLeftJoystickY,
                    xboxController::getRightJoystickY);
            m_arm.setDefaultCommand(m_basicArm);
        }

        if (SubsystemToggles.usePneumatics && SubsystemToggles.useController) {
            new Trigger(xboxController::getYButtonIsHeld).onTrue(new InstantCommand(m_compression::grabCone));
            new Trigger(xboxController::getXButtonIsHeld).onTrue(new InstantCommand(m_compression::grabCube));
            new Trigger(xboxController::getBButtonIsHeld).onTrue(new InstantCommand(m_compression::release));
        } else {
            System.out.println("Pneumatics or controller not operational");
        }

        if (SubsystemToggles.useDrive && SubsystemToggles.useController) {

            DriveWithJoystick driveWithJoystick = new DriveWithJoystick(
                    m_drivetrain,
                    m_controller::getWpiXAxis,
                    m_controller::getWpiYAxis,
                    m_controller::getWpiZAxis,
                    () -> m_controller.getRawButton(2));
    	    m_drivetrain.setDefaultCommand(driveWithJoystick);

            m_gyrocommand = new GyroCommand(m_drivetrain, 180);
            m_buttonboard = new ButtonBoard(0);

            // zero angle command when we are red make sure robot pointing forwards is 180
            new Trigger(() -> m_controller.getRawButton(1)).onTrue(new InstantCommand(() -> {
                double angle = (DriverStation.getAlliance() == DriverStation.Alliance.Red)?
                        180.0 : 0;
                m_navX.setAngle(angle);
            }));
            new Trigger(() -> m_controller.getRawButton(3)).onTrue(new InstantCommand(driveWithJoystick::toggleFieldRelative));
            new Trigger(() -> m_controller.getRawButton(4)).whileTrue(new GyroCommand(m_drivetrain, 180));
            new Trigger(() -> m_controller.getRawButton(5)).onTrue(new InstantCommand(() -> {
                m_drivetrain.getCurrentCommand().cancel();
                driveWithJoystick.schedule();
            }));

            //BUTTONBOARD TRIGGERS
            new Trigger(m_buttonboard::ChargeStationBalance).onTrue(new BalanceCommand(m_navX::getPitch,
                    m_navX::getRoll,
                    m_drivetrain));

        }

        if (SubsystemToggles.useDrive && driveType.equals("SwerveDrive")) {

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

            SmartDashboard.putData("Closest Blue alliance node", new DriveToScoringNode(
                    m_drivetrain, FieldConstants.Grids.blueAllianceGrid[0][0]));
            SmartDashboard.putData("Second Closest Blue alliance node", new DriveToScoringNode(
                    m_drivetrain, FieldConstants.Grids.blueAllianceGrid[1][0]));
        }
    }

    private Command getPathFollowCommand(PathPlannerTrajectory path) {
        return new PathFollowingCommand(m_drivetrain).withPath(path);
    }

    public Command getAutonomousCommand() {
        if (SubsystemToggles.useDrive && driveType.equals("SwerveDrive")) {
            var selectedPath = autoPathChooser.getSelected();
            m_robotState.setStartPose(selectedPath.path.getInitialHolonomicPose());
            return new FollowPathWithEvents(
                    getPathFollowCommand(selectedPath.path),
                    selectedPath.path.getMarkers(),
                    autoEventMap
            );
        }
        return new PrintCommand("Autonomous! -----");
    }
}

