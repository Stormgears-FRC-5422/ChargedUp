package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class DrivetrainBase extends SubsystemBase {
    /***
     * Use method to drive
     * @param vx meters per second
     * @param vy meters per second
     * @param rot radians per second
     */
    public abstract void drive(double vx, double vy, double rot);
    public void drive(ChassisSpeeds chassisSpeeds) {
        drive(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);
    }

    public abstract void zeroGyroScope();
    public abstract Rotation2d getGyroscopeRotation();
}
