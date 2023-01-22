package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class DrivetrainBase extends SubsystemBase {
    protected DrivetrainBase() {}
    public abstract void drive(ChassisSpeeds chassisSpeeds);
    public abstract void percentOutDrive(double tx, double ty, double rot);
    public abstract void zeroGyroscope();
    public abstract Rotation2d getGyroscopeRotation();
    public abstract Pose2d getPose();
    public abstract void setOdometry(Pose2d pose);

}
