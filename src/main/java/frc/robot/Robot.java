// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.IllegalDriveTypeException;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

import static frc.robot.Constants.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    public static boolean debug;
    private Command m_autonomousCommand;

    boolean lastEnabled = false;
    boolean enabled = false;

    private RobotContainer m_robotContainer;
    private RobotState m_state;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
      // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
      // autonomous chooser on the dashboard.

        m_state = RobotState.getInstance();

        System.out.println(
                "\n\n********************** \n" +
                "********************** \n" +
                "********************** \n" +
                "********************** \n" +
                "* Robot starting at "
                        + DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss").format(LocalDateTime.now()) + "\n" +
                "********************** \n" +
                "********************** \n" +
                "********************** \n" +
                "********************** \n\n\n");

        try {
          m_robotContainer = new RobotContainer();
        } catch (IllegalDriveTypeException e) {
          throw new RuntimeException(e);
        }

        System.out.println(
                "\n ************************** \n" +
                        "Timer at start " + m_state.getTimeSeconds() +
                        "\n *********************** \n");
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
        if (enabled && !lastEnabled) {
            m_robotContainer.onEnable();
        }
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        m_state.update();
        lastEnabled = enabled;
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        lastEnabled = false;
        enabled = false;
        m_robotContainer.onDisable();

        if(useCompressor) {
          m_robotContainer.compressor.stopCompressor();
          m_robotContainer.compressor.setPiston(false);
        }
    }

    @Override
    public void disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
      enabled = true;
      // schedule the autonomous command (example)
      m_autonomousCommand = m_robotContainer.getAutonomousCommand();
      if (m_autonomousCommand != null) {
          m_autonomousCommand.schedule();
      }

      System.out.println(
                "***************** \n" +
                "Auto Mode Start Time: " + m_state.getTimeSeconds()
                        + "\n" +
                "*****************");

      if(useCompressor) {
          m_robotContainer.compressor.setPiston(true);
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
         enabled = true;
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
          m_autonomousCommand.cancel();
        }
        System.out.println(
                "***************** \n" +
                        "Teleop Mode Start Time: " + m_state.getTimeSeconds()
                        + "\n" +
                        "*****************");

        if(useCompressor) {
          m_robotContainer.compressor.startCompressor();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();

        System.out.println(
                "***************** \n" +
                        "Test Mode Start Time: " + m_state.getTimeSeconds()
                        + "\n" +
                        "*****************");
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {

        System.out.println(
                "***************** \n" +
                        "Simulation Mode Start Time: " + m_state.getTimeSeconds()
                        + "\n" +
                        "*****************");
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}
}
