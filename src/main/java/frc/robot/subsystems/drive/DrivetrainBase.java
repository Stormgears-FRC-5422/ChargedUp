package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class DrivetrainBase extends SubsystemBase {

    public abstract void drive(double vx, double vy, double rot);
    public void drive(ChassisSpeeds chassisSpeeds) {}

    public abstract void zeroGyroScope();
    public abstract Rotation2d getGyroscopeRotation();
}
