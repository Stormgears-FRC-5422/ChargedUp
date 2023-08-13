// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.JoyDrive;
import frc.robot.subsystems.DriveSubsystem;
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

    //commands

    private JoyDrive joyDrive;

    

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        if (Constants.kUseDrive) {
            driveSubsystem = new DriveSubsystem();
            testWheel = new TestWheel();
        }

        if (Constants.kUseJoystick0) {
            xboxController = new StormXboxController(0);
        }

        if (Constants.kUseDrive && Constants.kUseJoystick0){
            joyDrive = new JoyDrive(driveSubsystem, xboxController);
            driveSubsystem.setDefaultCommand(joyDrive);
        }
        // Configure the trigger bindings
        configureBindings();

    }

    public DriveSubsystem getDriveSubsystem() {
        return driveSubsystem;
    }

    public StormXboxController getXboxController() {
        return xboxController;
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

    }
}
