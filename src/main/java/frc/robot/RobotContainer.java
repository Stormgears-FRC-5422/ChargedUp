// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.JoyDrive;
import frc.robot.commands.SetNeoPixels;
import frc.robot.commands.TestWheelSpeed;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NeoPixels;
import frc.robot.subsystems.TestWheel;
import frc.utils.joysticks.StormXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private StormXboxController xboxController;

    // The robot's subsystems and commands are defined here...

    private DriveSubsystem driveSubsystem;
    private TestWheel testWheel;
    private NeoPixels neoPixels;

    //commands

    private JoyDrive joyDrive;

    private TestWheelSpeed testWheelSpeed;

    private SetNeoPixels setNeoPixels;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        if (Constants.kUseDrive) {
            driveSubsystem = new DriveSubsystem();

        }

        if (Constants.kUseJoystick0) {
            xboxController = new StormXboxController(0);

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
            testWheel = new TestWheel();
            testWheelSpeed = new TestWheelSpeed(xboxController, testWheel);
            testWheel.setDefaultCommand(testWheelSpeed);
        }

        // Configure the trigger bindings
        configureBindings();

    }


    private void configureBindings() {

    }
}
