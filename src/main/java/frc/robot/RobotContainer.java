// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.subsystems.drive.DrivetrainFactory;
import frc.robot.subsystems.drive.IllegalDriveTypeException;
import frc.robot.subsystems.drive.SDSDrivetrain;
import frc.utils.joysticks.StormLogitechController;

import static frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  DrivetrainBase m_drivetrain;
  StormLogitechController m_controller;
  Trigger toggleFieldOrientedDrive;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() throws IllegalDriveTypeException {

    // Note the pattern of attempting to create the object then disabling it if that creation fails
    if (useDrive) {
      try {
        m_drivetrain = DrivetrainFactory.getInstance(driveType);
      } catch(Exception e) {
        e.printStackTrace();
        useDrive = false;
      }
    }

    // TODO - how do we know that this worked? e.g. what fails if the joystick is unplugged?
    if (useController) {
      m_controller = new StormLogitechController(kLogitechControllerPort);
      toggleFieldOrientedDrive = new Trigger(() -> m_controller.getRawButtonPressed(0));
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
      SlewRateLimiter forwardInputLimiter = new SlewRateLimiter(0.5);
      SlewRateLimiter sidewaysInputLimiter = new SlewRateLimiter(0.5);
      SlewRateLimiter rotationInputLimiter = new SlewRateLimiter(0.5);

//      m_drivetrain.setDefaultCommand(new DriveWithJoystick(
//              m_drivetrain,
//              () -> forwardInputLimiter.calculate(m_controller.getWpiXAxis()),
//              () -> sidewaysInputLimiter.calculate(m_controller.getWpiYAxis()),
//              () -> rotationInputLimiter.calculate(m_controller.getZAxis())
//      ));
        DriveWithJoystick driveWithJoystick = new DriveWithJoystick(
                m_drivetrain,
                () -> m_controller.getWpiXAxis(),
                () -> m_controller.getWpiYAxis(),
                () -> m_controller.getWpiZAxis()
        );
        m_drivetrain.setDefaultCommand(driveWithJoystick);

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
}
