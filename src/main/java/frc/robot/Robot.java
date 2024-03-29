// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.Constants;
import frc.robot.subsystems.NeoPixel;
import frc.robot.subsystems.drive.IllegalDriveTypeException;
import frc.utils.subsystemUtils.StormSubsystemScheduler;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    private int count = 0;
    private boolean wasEnabled = false;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.

        System.out.println(
                "\n\n********************** \n" +
                        "********************** \n" +
                        "* Robot starting at "
                        + DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss").format(LocalDateTime.now()) + "\n" +
                        "********************** \n" +
                        "********************** \n\n\n");

        try {
            m_robotContainer = new RobotContainer();
        } catch (IllegalDriveTypeException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // All of the enabled, disabled, auto, teleop init/periodic functions are
        // called before the subsystem periodics and command executes
        StormSubsystemScheduler.getInstance().run();
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

        if (++count % 1000 == 0) {
            System.out.println("total memory: " + Runtime.getRuntime().totalMemory() + " bytes");
        }

        if (DriverStation.isEnabled() && !wasEnabled) {
            m_robotContainer.enabledInit();
            wasEnabled = true;
        }
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        // schedule the autonomous command (example)

        RobotState.getInstance().setCurrentAlliance(DriverStation.getAlliance());
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
        m_robotContainer.m_aprilTagStatusCommand.schedule();
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        RobotState.getInstance().setCurrentAlliance(DriverStation.getAlliance());
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        //m_robotContainer.m_lidarIndicatorCommand.schedule();

        if (Constants.Toggles.useVision && Constants.Toggles.useStatusLights)
            m_robotContainer.m_aprilTagStatusCommand.schedule();


        if (Constants.Toggles.useButtonBoard) {
            if (Constants.Toggles.useNodeSelector) {
                if (m_robotContainer.m_buttonBoardConfig.topGrid()) {
                    m_robotContainer.nodeSelector.setSelectedRow(2);
                } else if (m_robotContainer.m_buttonBoardConfig.middleGrid()) {
                    m_robotContainer.nodeSelector.setSelectedRow(1);
                } else if (m_robotContainer.m_buttonBoardConfig.bottomGrid()) {
                    m_robotContainer.nodeSelector.setSelectedRow(0);
                }
            }

            if (Constants.Toggles.useStatusLights) {
                final boolean isCube = m_robotContainer.m_buttonBoardConfig.cubeSelected();
                RobotState.getInstance().setLidarRange(isCube? Constants.LidarRange.CUBE : Constants.LidarRange.CONE);
                m_robotContainer.m_neoPixel.setSpecificSegmentColor(m_robotContainer.allRingSegments,
                        isCube ? NeoPixel.PURPLE_COLOR : NeoPixel.YELLOW_COLOR);
            }
        }
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }

    /**
     * This function is called once when the robot is first started up.
     */
    @Override
    public void simulationInit() {
    }

    /**
     * This function is called periodically whilst in simulation.
     */
    @Override
    public void simulationPeriodic() {
    }
}
