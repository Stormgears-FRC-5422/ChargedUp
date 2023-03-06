package frc.robot.commands.drive;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DrivetrainBase;
import java.util.function.DoubleSupplier;


public class BalanceCommand extends CommandBase {

  private DoubleSupplier m_getPitch;

  private  DoubleSupplier m_getRoll;

  private DoubleSupplier getRoll;

  private  DoubleSupplier getPitch;


  private DrivetrainBase m_drive;

  private ChassisSpeeds m_speed = new ChassisSpeeds(
          0.0,
          0.,
          0.); ;

  private double balanceThresh;

  private final double Kp = 0.07;

  private double m_error;

  private double m_currTilt;

  private double balanceSpeed;


  public BalanceCommand(DoubleSupplier getPitch, DoubleSupplier getRoll, DrivetrainBase drive) {
    m_getPitch = getPitch;
    m_getRoll = getRoll;
    m_drive = drive;

    addRequirements(m_drive);
  }


  @Override
  public void initialize() {
    System.out.println("Balance Command Starting");
  }



  @Override
  public void execute() {
    System.out.println("Running");

    if(m_getPitch.getAsDouble() > 5) {
      System.out.println("Forward");
      m_currTilt = m_getPitch.getAsDouble();
      balanceThresh = 5;
    }
    if(m_getPitch.getAsDouble() < -5){
      m_currTilt = m_getPitch.getAsDouble();
      balanceThresh = -5;
      System.out.println("Backward");
    }

    if (m_getRoll.getAsDouble() > 5) {
      m_currTilt =m_getRoll.getAsDouble();
      balanceThresh = 5;
      System.out.println("Right");

    }
    if ( m_getRoll.getAsDouble() < -5) {
      m_currTilt =m_getRoll.getAsDouble();
      balanceThresh = -5;
      System.out.println("Left");

    }


    if(balanceThresh == 5){
    m_error =  m_currTilt - balanceThresh;
    }
    else{
      m_error = 5 + m_currTilt;
    }

    balanceSpeed = m_error * Kp;

    System.out.println("Balance Speed: " + balanceSpeed);
    System.out.println("Pitch/Roll: " + m_currTilt);

      m_drive.drive(
              new ChassisSpeeds(
                      balanceSpeed,
                      0.,
                      0.
              ),
              true
      );
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
