package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

import java.util.function.Supplier;

public class PoseEstimator extends SubsystemBase implements IEnabledDisabled {

    private SwerveDrivePoseEstimator m_poseEstimator;
    private final SwerveDriveKinematics m_driveKinematics;
    private final Supplier<SwerveModulePosition[]> m_modulePositionSupplier;

    private Pose2d m_currentPose;
    private Pose2d m_lastPose;

    public PoseEstimator(SwerveDriveKinematics kinematics,
                         Supplier<SwerveModulePosition[]> modulePositionSupplier) {

        m_driveKinematics = kinematics;
        m_modulePositionSupplier = modulePositionSupplier;
    }

    public void onEnable() {
        Pose2d m_startPose = RobotState.getInstance().getStartPose();
        m_currentPose = RobotState.getInstance().getCurrentPose();
        m_lastPose = RobotState.getInstance().getLastPose();

        m_poseEstimator = new SwerveDrivePoseEstimator(
                m_driveKinematics,
                m_startPose.getRotation(),
                m_modulePositionSupplier.get(),
                m_startPose
        );
    }

    @Override
    public void periodic() {
        //set last pose to uncalculated current pose
        m_lastPose = m_currentPose;

        var latestDriveEntry = RobotState.getInstance().getLatestDriveData();
        //drive data
        if (latestDriveEntry != null) {
            double time = latestDriveEntry.getKey();
            var data = latestDriveEntry.getValue();
            m_poseEstimator.updateWithTime(time, data.getGyroAngle(), data.getModulePositions());
        }

        var latestVisionEntry = RobotState.getInstance().getLatestVisionData();
        //add the vision entry to estimator
        if (latestVisionEntry != null) {
            double time = latestVisionEntry.getKey();
            Pose2d visionPose = latestVisionEntry.getValue();
            m_poseEstimator.addVisionMeasurement(visionPose, time);
        }

        //set pose in state object
        m_currentPose = m_poseEstimator.getEstimatedPosition();
        RobotState.getInstance().setCurrentPose(m_currentPose);
        RobotState.getInstance().setLastPose(m_lastPose);
    }

    private void resetEstimator(Rotation2d angle, SwerveModulePosition[] modulePositions, Pose2d pose) {
        m_poseEstimator.resetPosition(angle, modulePositions, pose);
    }

    private void resetEstimator(Pose2d pose) {
        resetEstimator(new Rotation2d(), new SwerveModulePosition[4], pose);
    }

    private void resetEstimator() {
        resetEstimator(new Pose2d());
    }
}
