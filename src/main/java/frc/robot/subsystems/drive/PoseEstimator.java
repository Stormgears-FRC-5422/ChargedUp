package frc.robot.subsystems.drive;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

import java.util.Map;

public class PoseEstimator extends SubsystemBase {

    private SwerveDrivePoseEstimator m_poseEstimator;
    private RobotState m_robotState;
    private Pose2d m_currentPose;
    private Pose2d m_lastPose;

    public PoseEstimator(SwerveDriveKinematics kinematics,
                         Rotation2d gyroAngle,
                         SwerveModulePosition[] modulePositions) {
        m_robotState = RobotState.getInstance();
        m_poseEstimator = new SwerveDrivePoseEstimator(kinematics,
                gyroAngle,
                modulePositions,
                m_robotState.getStartPose());
        m_currentPose = m_robotState.getStartPose();
        m_lastPose = m_currentPose;
    }

    @Override
    public void periodic() {
        m_lastPose = m_currentPose;
        Map.Entry<Double, RobotState.DriveData> latestDriveEntry = m_robotState.getLatestDriveData();
        //drive data
        if (latestDriveEntry != null) {
            double time = latestDriveEntry.getKey();
            RobotState.DriveData data = latestDriveEntry.getValue();
            m_poseEstimator.updateWithTime(time, data.getGyroAngle(), data.getModulePositions());
        }

        //set pose in state object
        m_currentPose = m_poseEstimator.getEstimatedPosition();
        m_robotState.setCurrentPose(m_currentPose);
    }

    public void resetEstimator(RobotState.DriveData data, Pose2d pose) {
        m_poseEstimator.resetPosition(data.getGyroAngle(), data.getModulePositions(), pose);
    }

    public void resetEstimator(Pose2d pose) {
        resetEstimator(new RobotState.DriveData(), pose);
    }

    public void resetEstimator() {
        resetEstimator(new Pose2d());
    }

    public double getDeltaDistance() {
        Translation2d currentTranslation = m_currentPose.getTranslation();
        Translation2d lastTranslation = m_lastPose.getTranslation();
        return currentTranslation.getDistance(lastTranslation);
    }
}
