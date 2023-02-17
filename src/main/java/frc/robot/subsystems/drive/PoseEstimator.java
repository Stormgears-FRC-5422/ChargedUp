package frc.robot.subsystems.drive;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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

        ShuffleboardTab mainTab = Shuffleboard.getTab("MainTab");
    }

    public void onEnable() {
        resetEstimator();
    }

    public void onDisable() {
    }

    @Override
    public void periodic() {
        m_lastPose = m_currentPose;
        var latestDriveEntry = m_robotState.getLatestDriveData();
        //drive data
        if (latestDriveEntry != null) {
            double time = latestDriveEntry.getKey();
            var data = latestDriveEntry.getValue();
            m_poseEstimator.updateWithTime(time, data.getGyroAngle(), data.getModulePositions());
        }

        //visiondata
//        var latestVisionEntry = m_robotState.getLatestVisionData();
//        if (latestVisionEntry != null) {
//            double time = latestVisionEntry.getKey();
//            var data = latestVisionEntry.getValue();
//            m_poseEstimator.addVisionMeasurement(data, time);
//        }

        //set pose in state object
        m_currentPose = m_poseEstimator.getEstimatedPosition();
        m_robotState.setCurrentPose(m_currentPose);
        m_robotState.setLastPose(m_lastPose);
    }

    private void resetEstimator(RobotState.DriveData data, Pose2d pose) {
        m_poseEstimator.resetPosition(data.getGyroAngle(), data.getModulePositions(), pose);
    }

    private void resetEstimator(Pose2d pose) {
        resetEstimator(new RobotState.DriveData(), pose);
    }

    private void resetEstimator() {
        resetEstimator(new Pose2d());
    }
}
