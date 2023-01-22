package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

import static frc.robot.Constants.*;

public class MecanumDrivetrain extends DrivetrainBase {

    private final WPI_TalonSRX m_frontLeftTalon;
    private final WPI_TalonSRX m_frontRightTalon;
    private final WPI_TalonSRX m_backLeftTalon;
    private final WPI_TalonSRX m_backRightTalon;

    private final MecanumDrive m_drive;

    public MecanumDrivetrain() {
        m_frontLeftTalon = new WPI_TalonSRX(frontLeftEncoderID);
        m_frontRightTalon = new WPI_TalonSRX(frontRightEncoderID);
        m_backLeftTalon = new WPI_TalonSRX(backLeftEncoderID);
        m_backRightTalon = new WPI_TalonSRX(backRightEncoderID);
        //Fix
        m_frontRightTalon.setInverted(true);
        m_backRightTalon.setInverted(true);

        m_drive = new MecanumDrive(m_frontLeftTalon, m_backLeftTalon, m_frontRightTalon, m_backRightTalon);
    }

    @Override
    public void periodic() {


    }

    @Override
    public void drive(ChassisSpeeds chassisSpeeds) {
    }
    @Override
    public void percentOutDrive(double tx, double ty, double rot) {
        m_drive.driveCartesian(tx, ty, rot);
    }
    @Override
    public Pose2d getPose() {
        return null;
    }

    @Override
    public void setOdometry(Pose2d pose) {

    }
}
