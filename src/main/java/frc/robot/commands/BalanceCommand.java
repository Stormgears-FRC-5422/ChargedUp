package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.subsystems.NavX;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.navXConnection;
import static java.lang.Math.abs;


public class BalanceCommand extends CommandBase {

  private DrivetrainBase m_drivetrain;

  private NavX m_NavX;
  private DoubleSupplier m_getPitch;

  private  DoubleSupplier m_getRoll;

  private DoubleSupplier getRoll;

  private  DoubleSupplier getPitch;


  private DrivetrainBase m_drive;

  private ChassisSpeeds m_speed = new ChassisSpeeds(
          0.0,
          0.,
          0.); ;

  private double balanceThresh = 5;

  private final double Kp = 0.07;

  private double m_error;

  private double m_currTilt;

  private double balanceSpeed;

  private String direction;


  public BalanceCommand(DoubleSupplier getPitch, DoubleSupplier getRoll, DrivetrainBase drive) {
    m_getPitch = getPitch;
    m_getRoll = getRoll;
    m_drive = drive;

    addRequirements(m_drive);

  }


  @Override
  public void initialize() {
    System.out.println("Balance Command Starting");
    if(m_getPitch.getAsDouble() > 5) {
      direction = "Forward";
    }
    if (m_getPitch.getAsDouble() < -5) {
      direction = "Backward";
    }
    if (m_getRoll.getAsDouble() > 5) {
      direction = "Right";
    }
    if (m_getRoll.getAsDouble() < -5) {
      direction = "Left";
    }

  }



  @Override
  public void execute() {
    System.out.println("Running");

    switch (direction) {
      case "Forward":
      case "Backward":
        m_currTilt = m_getPitch.getAsDouble();
        System.out.println(m_getPitch.getAsDouble() + " Pitch");
        break;
      case "Right":
      case "Left":
        m_currTilt = m_getRoll.getAsDouble();
        System.out.println(m_getRoll.getAsDouble() + " Roll");
        break;
    }

    m_error =  balanceThresh - m_currTilt;
    balanceSpeed = m_error * Kp;
    System.out.println(balanceSpeed);

    if(abs(m_currTilt) > balanceThresh) {
      m_drive.drive(
              new ChassisSpeeds(
                      balanceSpeed,
                      0.,
                      0.
              ),
              true
      );
    }
  }


  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    System.out.println("Balance Command ending");
    m_drive.stopDrive();
  }

  @Override
  public boolean isFinished() {
    boolean finished = (m_getPitch.getAsDouble() < 5 ) && (m_getPitch.getAsDouble() > -5 )
            && (m_getRoll.getAsDouble() < 5 ) && (m_getRoll.getAsDouble() > -5 );
    System.out.println(finished);
    return finished;
  }
}
