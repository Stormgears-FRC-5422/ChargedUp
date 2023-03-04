// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.MecanumDrive;
import frc.robot.subsystems.NeoPixel;
import frc.robot.subsystems.Vision;
import frc.utils.joysticks.StormLogitechController;
import frc.robot.commands.JoystickDrive;
import frc.utils.joysticks.ButtonBoard;

import javax.swing.plaf.synth.SynthOptionPaneUI;
import java.awt.*;

import static frc.robot.subsystems.NeoPixel.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  NavX m_gyro;


  Vision m_vision_ss;

  ButtonBoard m_buttonboard;

  NeoPixel neoPixel;

  public Color8Bit m_fullColor;

    public void myFunction () {
      neoPixel.setAll(m_fullColor);
    }

//public void myFunction() {
//  neoPixel.setAll(fullColor);
//}



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


    neoPixel = new NeoPixel();

    m_fullColor = fullColor;


//    Joystick joystick = new Joystick(3);
//    JoystickButton joystickButton = new JoystickButton(joystick, 4);
//
//    new Trigger(() -> joystick.getRawButton(4)).whileTrue(System.out.println("working"));

    // Configure the trigger bindings
    configureBindings();
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
    Joystick controller = new Joystick(0);
    System.out.println("about to start trigger");
    new Trigger(() -> controller.getRawButton(3)).whileTrue(new InstantCommand(() -> neoPixel.setSecondRing(fullColorY)));
    new Trigger(() -> controller.getRawButton(3)).whileFalse(new InstantCommand(() -> neoPixel.setSecondRing(blankColor)));
    new Trigger(() -> controller.getRawButton(2)).whileTrue(new InstantCommand(() -> neoPixel.setFirstRing(fullColor)));
    new Trigger(() -> controller.getRawButton(2)).whileFalse(new InstantCommand(() -> neoPixel.setFirstRing(blankColor)));

    //added for light tower
    new Trigger(() -> controller.getRawButton(4)).whileTrue(new InstantCommand(() -> neoPixel.setThirdRing(fullColor)));
    new Trigger(() -> controller.getRawButton(4)).whileFalse(new InstantCommand(() -> neoPixel.setThirdRing(blankColor)));

    ButtonBoard m_buttonboard1 = new ButtonBoard(1);
    ButtonBoard m_buttonboard2 = new ButtonBoard(2);

    if(!m_buttonboard1.jumper()){
      System.out.println("Switching ButtonBoard ports");
     m_buttonboard1 = new ButtonBoard(2);
     m_buttonboard2 = new ButtonBoard(1);
    }
    else {
      System.out.println("Not Swithcing ButtonBoard Port");
    }

    new Trigger(m_buttonboard1::button3).whileTrue(new InstantCommand(() -> neoPixel.setSecondRing(fullColorY)));
    new Trigger(m_buttonboard1::button3).whileFalse(new InstantCommand(() -> neoPixel.setSecondRing(blankColor)));
    new Trigger(m_buttonboard2::button3).whileTrue(new InstantCommand(() -> neoPixel.setFirstRing(fullColor)));
    new Trigger(m_buttonboard2::button3).whileFalse(new InstantCommand(() -> neoPixel.setFirstRing(blankColor)));


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
