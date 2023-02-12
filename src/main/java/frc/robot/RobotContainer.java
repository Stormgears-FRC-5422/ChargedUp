// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.DriveWithJoystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TrapezoidMoveForward;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.Compression;
import frc.robot.commands.GyroCommand;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.subsystems.drive.DrivetrainFactory;

import frc.robot.subsystems.drive.IllegalDriveTypeException;
import frc.utils.joysticks.StormLogitechController;

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
  PoseEstimator m_poseEstimator;

  public Compression compressor;

  StormLogitechController m_controller;

  private ShuffleboardTab mainTab;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() throws IllegalDriveTypeException {

    mainTab = Shuffleboard.getTab("Main");
    m_robotState = RobotState.getInstance();

    // Note the pattern of attempting to create the object then disabling it if that creation fails
    if (useDrive) {
      if (driveType.equals("SwerveDrive")) {
        m_poseEstimator = new PoseEstimator(
                m_drivetrain.getSwerveDriveKinematics(),
                m_drivetrain.getGyroscopeRotation(),
                m_drivetrain.getSwerveModulePositions());
        m_robotState.setStartPose(new Pose2d());
      }

      try {
        m_drivetrain = DrivetrainFactory.getInstance(driveType);
      } catch(Exception e) {
        e.printStackTrace();
        useDrive = false;
        System.out.println("NOT using drive - caught exception!");
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

        new Trigger(() -> m_controller.getRawButton(1)).onTrue(new InstantCommand(()-> m_drivetrain.zeroGyroscope()));
        new Trigger(() -> m_controller.getRawButton(3)).onTrue(new InstantCommand(driveWithJoystick::toggleFieldRelative));
        new Trigger(() -> m_controller.getRawButton(4)).whileTrue(new GyroCommand(m_drivetrain, 180));
    }

    if (useDrive) {
      SmartDashboard.putData("Trapezoid Move Forward Command", new TrapezoidMoveForward(m_drivetrain, m_poseEstimator, 1, 1, 0.1));
    }
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

  PoseEstimator getPoseEstimator() {
    return m_poseEstimator;
  }
}

