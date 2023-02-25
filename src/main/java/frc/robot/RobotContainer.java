// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.MecanumDrive;
import frc.robot.subsystems.NeoPixel;
import frc.robot.subsystems.Vision;
import frc.utils.joysticks.StormLogitechController;
import frc.robot.commands.JoystickDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  NavX m_gyro;
  Vision m_vision_ss;
//
//  StormLogitechController m_controller = new StormLogitechController(0);
//  private final MecanumDrive mecanumdrive = new MecanumDrive();
//  Trigger zeroGyroTrigger = new Trigger(() -> m_controller.getRawButton(1));
//
//  private final JoystickDrive joystickDrive = new JoystickDrive(mecanumdrive, m_controller::getYAxis,
//          m_controller::getXAxis, m_controller::getZAxis, m_controller::getSliderAxis);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {


    m_gyro = new NavX();
    m_vision_ss = new Vision();

    NeoPixel neoPixel = new NeoPixel();
    // Configure the trigger bindings
//    configureBindings();
//
//    mecanumdrive.setDefaultCommand(joystickDrive);
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
//    zeroGyroTrigger.onTrue(new InstantCommand(() -> m_gyro.zeroYaw()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new InstantCommand(() -> System.out.println("auto"));
  }
}
