package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import frc.robot.RobotState;
import frc.robot.constants.Constants;
import frc.robot.constants.ShuffleboardConstants;
import frc.robot.subsystems.vision.AprilTagPoseEstimationStrategy;
import frc.robot.subsystems.vision.Vision;
import frc.utils.subsystemUtils.StormSubsystemBase;

import java.util.Arrays;
import java.util.Vector;

import static frc.robot.constants.Constants.VisionConstants.*;

public class PoseEstimator extends StormSubsystemBase {
    private final SwerveDrivePoseEstimator m_poseEstimator;

    private Pose2d m_currentPose;

    private final FieldObject2d
            visionPoseSim,
            estimatedPoseSim;

    // These standard deviations determine how much the pose estimation trusts itself (state)
    // and how much it trusts the vision measurements in (x, y, radians)
    // increase these to trust less
    // TODO: tune these numbers
    private final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    private final Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);

    private double currentOdometryEntryTime, currentVisionEntryTime;

    public PoseEstimator(SwerveDriveKinematics kinematics,
                         SwerveModulePosition[] modulePositions) {
        var startPose = RobotState.getInstance().getStartPose();

        m_poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                RobotState.getInstance().getCurrentGyroRotation(),
                modulePositions,
                startPose,
                stateStdDevs,
                visionMeasurementStdDevs
        );

        Field2d fieldSim = ShuffleboardConstants.getInstance().poseEstimationFieldSim;
        visionPoseSim = fieldSim.getObject("Vision Pose");
        estimatedPoseSim = fieldSim.getRobotObject();
    }

    @Override
    public void enabledInit() {
        Pose2d startPose = RobotState.getInstance().getStartPose();
        System.out.println("Start pose at pose estimator on enable: " + startPose);
        resetEstimator(startPose);
    }

    @Override
    public void enabledPeriodic() {
        //set last pose to uncalculated current pose
        Pose2d lastPose = m_currentPose;

        var currentOdometryData = RobotState.getInstance().getCurrentOdometryData();
        //drive data
        if (currentOdometryData != null) {
            double time = currentOdometryData.getFirst();
            if (time != currentOdometryEntryTime) {
                var data = currentOdometryData.getSecond();
                m_poseEstimator.updateWithTime(time, data.getGyroAngle(), data.getModulePositions());
                currentOdometryEntryTime = time;
            }
        }

        if (Constants.SubsystemToggles.useVision) {
            var currentVisionData = RobotState.getInstance().getCurrentVisionData();
            //add the vision entry to estimator
            if (currentVisionData != null) {
                double time = currentVisionData.getFirst();
                if (time != currentVisionEntryTime) {
                    var info = currentVisionData.getSecond();
                    for (var tag : info) {
                        System.out.println(tag.toString());
                    }
                    // calculate camera angle by adding to the gyro angle
                    Rotation2d gyroAngle = RobotState.getInstance().getAngleAtTimeSeconds(time);
                    Rotation2d cameraAngle = gyroAngle.rotateBy(CAMERA_POSITION.getRotation().toRotation2d());
                    // just call the pose estimation strategy class
                    Pose2d camPose = AprilTagPoseEstimationStrategy.fromAprilTagData(info, cameraAngle);
                    // have to transform to robot pose
                    Pose2d visionRobotPose = camPose.transformBy(CAMERA_ROBOT_TRANSFORM2D);
                    m_poseEstimator.addVisionMeasurement(visionRobotPose, time);
                    // log the position
                    visionPoseSim.setPose(visionRobotPose);
                    currentVisionEntryTime = time;
                }
            }
        }

        //set pose in state object
        m_currentPose = m_poseEstimator.getEstimatedPosition();
        estimatedPoseSim.setPose(m_currentPose);
        RobotState.getInstance().setCurrentPose(m_currentPose);
        RobotState.getInstance().setLastPose(lastPose);
    }

    public void resetEstimator(Rotation2d angle, SwerveModulePosition[] modulePositions, Pose2d pose) {
        m_poseEstimator.resetPosition(angle, modulePositions, pose);
    }

    public void resetEstimator(Pose2d pose) {
        resetEstimator(
                RobotState.getInstance().getCurrentGyroRotation(),
                new SwerveModulePosition[] {
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                        new SwerveModulePosition()
                },
                pose
        );
    }
}
