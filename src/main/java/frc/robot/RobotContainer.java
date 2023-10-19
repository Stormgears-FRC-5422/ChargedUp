// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.JoyDrive;
import frc.robot.commands.SetNeoPixels;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.ShooterTestCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NeoPixels;
import frc.robot.subsystems.ShooterSubsystem;
import frc.utils.joysticks.StormXboxController;

import static frc.utils.joysticks.StormXboxController.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    private StormXboxController xboxController;
    private JoystickButton lightsButton;
    private JoystickButton intakeButton;
    private JoystickButton shooterButton;
    private JoystickButton shootertestButton;
    // The robot's subsystems and commands are defined here...

    private DriveSubsystem driveSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private NeoPixels neoPixels;

    //commands

    private JoyDrive joyDrive;

    private ShooterCommand shooterCommand;

    private IntakeCommand intakeCommand;

    private SetNeoPixels setNeoPixels;

    private ShooterTestCommand shooterTestCommand;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        if (Constants.kUseDrive) {
            driveSubsystem = new DriveSubsystem();

        }

        if (Constants.kUseJoystick0) {
            xboxController = new StormXboxController(0);
            lightsButton = new JoystickButton(xboxController, AButton);
            intakeButton = new JoystickButton(xboxController, rightBumper);
            shooterButton = new JoystickButton(xboxController, XButton);
            shootertestButton = new JoystickButton(xboxController, YButton);

        }

        if (Constants.kUseDrive && Constants.kUseJoystick0) {
            joyDrive = new JoyDrive(driveSubsystem, xboxController);
            driveSubsystem.setDefaultCommand(joyDrive);

        }

        if (Constants.kUseLights) {
            neoPixels = new NeoPixels();
            setNeoPixels = new SetNeoPixels(xboxController, neoPixels);
            neoPixels.setDefaultCommand(setNeoPixels);

        }

        if (Constants.kUseShooter && Constants.kUseJoystick0) {
            shooterSubsystem = new ShooterSubsystem();
            intakeCommand = new IntakeCommand(shooterSubsystem);
            shooterCommand = new ShooterCommand(shooterSubsystem, xboxController);
            shooterTestCommand = new ShooterTestCommand(shooterSubsystem, xboxController);
            configureBindings();


        }


        // Configure the trigger bindings

    }


    private void configureBindings() {
        System.out.println("configure bindings running");
        if (Constants.kUseShooter) {
            intakeButton.whileTrue(intakeCommand);
            shooterButton.whileTrue(shooterCommand);
            shootertestButton.whileTrue(shooterTestCommand);
            System.out.println("use shooter check running");

        }
    }
}
