package frc.robot.subsystems.drive.SwerveModules;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleBase {
    double getPosition();
    double getDriveVel();
    Rotation2d getSteerAngle();
    void setDesiredState(double speed, Rotation2d angle);
}
