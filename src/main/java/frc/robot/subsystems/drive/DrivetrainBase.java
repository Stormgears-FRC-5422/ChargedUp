package frc.robot.subsystems.drive;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.RobotState;
import frc.robot.constants.ShuffleboardConstants;
import frc.utils.subsystemUtils.StormSubsystemBase;

import static frc.robot.constants.Constants.kDriveSpeedScale;

public abstract class DrivetrainBase extends StormSubsystemBase {

    public double m_maxVelocityMetersPerSecond;
    public double m_maxAngularVelocityRadiansPerSecond;
    protected double m_driveSpeedScale = 0;

    protected ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    protected final ShuffleboardTab tab;
    DrivetrainBase() {
        setDriveSpeedScale(kDriveSpeedScale);
        tab = ShuffleboardConstants.getInstance().drivetrainTab;
    }

    protected void setMaxVelocities(double maxVelocityMetersPerSecond, double maxAngularVelocityRadiansPerSecond) {
        this.m_maxVelocityMetersPerSecond = maxVelocityMetersPerSecond;
        this.m_maxAngularVelocityRadiansPerSecond = maxAngularVelocityRadiansPerSecond;
    }

    public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
        // Scale incoming speeds
        ChassisSpeeds s = new ChassisSpeeds(
                m_driveSpeedScale * speeds.vxMetersPerSecond,
                m_driveSpeedScale * speeds.vyMetersPerSecond,
                m_driveSpeedScale * speeds.omegaRadiansPerSecond);

        if (fieldRelative) {
            m_chassisSpeeds =
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            s,
                            RobotState.getInstance().getCurrentPose().getRotation());
        }
        else {
            m_chassisSpeeds = s;
        }
    }

    public void percentOutDrive(ChassisSpeeds speeds, boolean fieldRelative) {
        drive(new ChassisSpeeds(speeds.vxMetersPerSecond * m_maxVelocityMetersPerSecond,
                        speeds.vyMetersPerSecond * m_maxVelocityMetersPerSecond,
                        speeds.omegaRadiansPerSecond * m_maxAngularVelocityRadiansPerSecond),
                fieldRelative);
    }

    public void setDriveSpeedScale(double scale) {
        m_driveSpeedScale = MathUtil.clamp(scale, 0, kDriveSpeedScale);
    }

    public void stopDrive() {
        drive(new ChassisSpeeds(0, 0, 0), false);
    }

    protected Rotation2d getGyroscopeRotation() {
        return RobotState.getInstance().getCurrentGyroRotation();
    }

    public ChassisSpeeds getCurrentChassisSpeeds() {
        return m_chassisSpeeds;
    }

    public SwerveDriveKinematics getSwerveDriveKinematics() {return new SwerveDriveKinematics();}

    public SwerveModulePosition[] getSwerveModulePositions() {return new SwerveModulePosition[4];}
    public void goToPPTrajectoryState(PathPlannerTrajectory.PathPlannerState goalState) {}
}