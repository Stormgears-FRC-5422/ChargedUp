package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.networktables.*;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private WPI_TalonSRX m_Left0;
  private WPI_TalonSRX m_Left1;
  private WPI_TalonSRX m_Right0;
  private WPI_TalonSRX m_Right1;
  private MecanumDrive m_Drive;

  private XboxController m_Controller = new XboxController(0);

  private boolean m_LimelightHasValidTarget = false;
  private double m_LimelightDriveCommand = 0.0;
  private double m_LimelightSteerCommand = 0.0;
  private int SpeakerMidAprilTag;
  private int SpeakerSideAprilTag;
  private int SourceLeftAprilTag;
  private int SourceRightAprilTag;
  private int AMPAprilTag;
  private int Podium1AprilTag;
  private int Podium2AprilTag;
  private int Podium3AprilTag;
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red){
      SpeakerMidAprilTag = 4;
      SpeakerSideAprilTag = 3;
      SourceLeftAprilTag = 10;
      SourceRightAprilTag = 9;
      AMPAprilTag = 5;
      Podium1AprilTag = 13;
      Podium2AprilTag = 12;
      Podium3AprilTag = 11;
    } else if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
      SpeakerMidAprilTag = 7;
      SpeakerSideAprilTag = 8;
      SourceLeftAprilTag = 2;
      SourceRightAprilTag = 1;
      AMPAprilTag = 6;
      Podium1AprilTag = 14;
      Podium2AprilTag = 15;
      Podium3AprilTag = 16;
    }


    m_Left0 = new WPI_TalonSRX(1);
    m_Left1 = new WPI_TalonSRX(3);
    m_Right0 = new WPI_TalonSRX(6);
    m_Right1 = new WPI_TalonSRX(4);
    m_Right0.setInverted(true);
    m_Right1.setInverted(true);
    m_Drive = new MecanumDrive(m_Left0, m_Left1, m_Right0, m_Right1);
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {


    Update_Limelight_Tracking();

    double steer = m_Controller.getRightX();
    double drive = -m_Controller.getLeftY();
    boolean auto = m_Controller.getAButton();
    steer *= 0.70;
    drive *= 0.70;


    if (auto)
    {
      System.out.println("auto on");
      if (m_LimelightHasValidTarget)
      {
        System.out.println("has valid target " + m_LimelightDriveCommand + " " + m_LimelightSteerCommand);
        m_Drive.driveCartesian(0.0, m_LimelightDriveCommand, m_LimelightSteerCommand);

      }
      else
      {
        System.out.println("no valid target " + m_LimelightDriveCommand + " " + m_LimelightSteerCommand);
        m_Drive.driveCartesian(0.0, 0.0, 0.0);
      }
    }
    else
    {
      System.out.println("not auto");
      m_Drive.driveCartesian(drive, 0.0, steer);
    }
  }

  @Override
  public void testPeriodic() {
  }



  /**
   * This function implements a simple method of generating driving and steering commands
   * based on the tracking data from a limelight camera.
   */
  public void Update_Limelight_Tracking()
  {
    // These numbers must be tuned for your Robot!  Be careful!
    final double STEER_K = 0.03;                    // how hard to turn toward the target
    final double DRIVE_K = 0.05;                    // how hard to drive fwd toward the target
    final double DESIRED_TARGET_AREA = 13.0;        // Area of the target when the robot reaches the wall
    final double MAX_DRIVE = 0.1;                   // Simple speed limit so we don't drive too fast

    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

    if (tv < 1.0)
    {
      m_LimelightHasValidTarget = false;
      m_LimelightDriveCommand = 0.0;
      m_LimelightSteerCommand = 0.0;
      return;
    }

    m_LimelightHasValidTarget = true;

    // Start with proportional steering
    double steer_cmd = tx * STEER_K;
    m_LimelightSteerCommand = steer_cmd;
    if (steer_cmd<0) {
      steer_cmd=0;
    }

    // try to drive forward until the target area reaches our desired area
    double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

    // don't let the robot drive too fast into the goal
    if (drive_cmd > MAX_DRIVE)
    {
      drive_cmd = MAX_DRIVE;
    }
    if (drive_cmd<0) {
      drive_cmd = 0;
    }

    m_LimelightDriveCommand = drive_cmd;
  }
}
