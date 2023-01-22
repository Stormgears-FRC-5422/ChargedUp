package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.NavX;

public abstract class DrivetrainBase extends SubsystemBase {
    protected NavX m_gyro = new NavX();
    protected DrivetrainBase() {}
    public abstract void drive(ChassisSpeeds chassisSpeeds);
    public abstract void percentOutDrive(double tx, double ty, double rot);
    public void zeroGyroscope() {
        m_gyro.zeroYaw();
    }
    public Rotation2d getGyroscopeRotation() {
        if (m_gyro.isMagnetometerCalibrated()) {
            // We will only get valid fused headings if the magnetometer is calibrated
            return Rotation2d.fromDegrees(m_gyro.getFusedHeading());
        }

        // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
        return Rotation2d.fromDegrees(360.0 - m_gyro.getYaw());
    }
    public abstract Pose2d getPose();
    public abstract void setOdometry(Pose2d pose);

}
