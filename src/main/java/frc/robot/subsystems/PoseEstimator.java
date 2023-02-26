package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

import java.util.function.Supplier;

public class PoseEstimator extends SubsystemBase implements IEnabledDisabled {

    private SwerveDrivePoseEstimator m_poseEstimator;
    private final SwerveDriveKinematics m_driveKinematics;
    private final Supplier<SwerveModulePosition[]> m_modulePositionSupplier;

    private Pose2d m_currentPose, m_lastPose;

    private final ShuffleboardTab tab = Shuffleboard.getTab("Pose Estimation");
    private final Field2d fieldSim;
    private final FieldObject2d odometryPoseSim, visionPoseSim, estimatedPoseSim;

    // These standard deviations determine how much the pose estimation trusts itself (state)
    // and how much it trusts the vision measurements in (x, y, radians)
    // increase these to trust less
    // TODO: tune these numbers
    private final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    private final Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);

    public PoseEstimator(SwerveDriveKinematics kinematics,
                         Supplier<SwerveModulePosition[]> modulePositionSupplier) {

        m_driveKinematics = kinematics;
        m_modulePositionSupplier = modulePositionSupplier;

        fieldSim = new Field2d();
        tab.add("Field", fieldSim)
                .withWidget(BuiltInWidgets.kField)
                .withSize(5, 4).withPosition(0, 0);
        odometryPoseSim = fieldSim.getObject("Odometry Pose");
        visionPoseSim = fieldSim.getObject("Vision Pose");
        estimatedPoseSim = fieldSim.getRobotObject();
    }

    @Override
    public void onEnable() {
        Pose2d m_startPose = RobotState.getInstance().getStartPose();
        m_currentPose = RobotState.getInstance().getCurrentPose();
        m_lastPose = RobotState.getInstance().getLastPose();

        m_poseEstimator = new SwerveDrivePoseEstimator(
                m_driveKinematics,
                RobotState.getInstance().getCurrentGyroRotation(),
                m_modulePositionSupplier.get(),
                m_startPose,
                stateStdDevs,
                visionMeasurementStdDevs
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
            odometryPoseSim.setPose(m_poseEstimator.getEstimatedPosition());
        }

        var latestVisionEntry = RobotState.getInstance().getLatestVisionData();
        //add the vision entry to estimator
        if (latestVisionEntry != null) {
            double time = latestVisionEntry.getKey();
            Pose2d pose = latestVisionEntry.getValue();
            m_poseEstimator.addVisionMeasurement(pose, time);
            visionPoseSim.setPose(pose);
        }

        //set pose in state object
        m_currentPose = m_poseEstimator.getEstimatedPosition();
        estimatedPoseSim.setPose(m_currentPose);
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
