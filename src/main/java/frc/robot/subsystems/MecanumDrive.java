package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MecanumDrive extends SubsystemBase {

  private final WPI_TalonSRX frontLeftTalon;
  private final WPI_TalonSRX frontRightTalon;
  private final WPI_TalonSRX backLeftTalon;
  private final WPI_TalonSRX backRightTalon;

  private final edu.wpi.first.wpilibj.drive.MecanumDrive mecanumDrive;

  /** Creates a new ExampleSubsystem. */
  public MecanumDrive() {
    frontLeftTalon = new WPI_TalonSRX(Constants.frontLeftTalonID);
    frontRightTalon = new WPI_TalonSRX(Constants.frontRightTalonID);
    backLeftTalon = new WPI_TalonSRX(Constants.backLeftTalonID);
    backRightTalon = new WPI_TalonSRX(Constants.backRightTalonID);

    frontRightTalon.setInverted(true);
    backRightTalon.setInverted(true);

    mecanumDrive = new edu.wpi.first.wpilibj.drive.MecanumDrive(frontLeftTalon, backLeftTalon, frontRightTalon, backRightTalon);
  }

  public void drive(double y, double x, double z) {
    mecanumDrive.driveCartesian(y, x, z);
  }

}