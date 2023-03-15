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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import frc.robot.RobotState;
import frc.robot.constants.Constants;
import frc.robot.constants.ShuffleboardConstants;
import frc.robot.subsystems.vision.AprilTagPoseEstimationStrategy;
import frc.utils.subsystemUtils.StormSubsystemBase;

import static frc.robot.constants.Constants.VisionConstants.CAMERA_POSITION;
import static frc.robot.constants.Constants.VisionConstants.CAMERA_ROBOT_TRANSFORM2D;

public class PoseEstimator extends StormSubsystemBase {
    private final SwerveDrivePoseEstimator m_poseEstimator;

    private Pose2d currentPose;
    private Pose2d visionPose;

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

    public PoseEstimator(SwerveDriveKinematics kinematics) {
        var startPose = RobotState.getInstance().getStartPose();

        var data = RobotState.getInstance().getCurrentOdometryData().getSecond();
        m_poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                data.getGyroAngle(),
                data.getModulePositions(),
                startPose,
                stateStdDevs,
                visionMeasurementStdDevs
        );

        Field2d fieldSim = ShuffleboardConstants.getInstance().poseEstimationFieldSim;
        visionPoseSim = fieldSim.getObject("Vision Pose");
        estimatedPoseSim = fieldSim.getRobotObject();

        ShuffleboardLayout layout = ShuffleboardConstants.getInstance().robotStateList;
        layout.addDouble("Vision X", () -> getVisionPose().getX()).withPosition(0, 0);
        layout.addDouble("Vision Y", () -> getVisionPose().getY()).withPosition(0, 1);
        layout.addDouble("Vision Rotation", () -> getVisionPose().getRotation().getDegrees())
                .withPosition(0, 2);
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
        Pose2d lastPose = currentPose;

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

        if (Constants.Toggles.useVision && RobotState.getInstance().getCurrentLinearVel() <= 2.5) {
            var currentVisionData = RobotState.getInstance().getCurrentVisionData();
            //add the vision entry to estimator
            if (currentVisionData != null) {
                double time = currentVisionData.getFirst();
                if (time != currentVisionEntryTime) {
                    var info = currentVisionData.getSecond();
                    // calculate camera angle by adding to the gyro angle
                    Rotation2d robotRotation = RobotState.getInstance().getRotationAtTime(time);
                    Rotation2d cameraAngle = robotRotation.rotateBy(CAMERA_POSITION.getRotation().toRotation2d());
                    // just call the pose estimation strategy class
                    Pose2d camPose = AprilTagPoseEstimationStrategy.fromAprilTagData(info, cameraAngle);
                    // have to transform to robot pose
                    visionPose = camPose.transformBy(CAMERA_ROBOT_TRANSFORM2D);
                    if (RobotState.getInstance().getCurrentLinearVel() <= 0.15)
                        resetEstimator(visionPose);
//                    System.out.println(visionPose);
                    m_poseEstimator.addVisionMeasurement(visionPose, time);
                    // log the position
                    visionPoseSim.setPose(visionPose);
                    currentVisionEntryTime = time;
                }
            }
        }

        //add pose to state object
        currentPose = m_poseEstimator.getEstimatedPosition();
        estimatedPoseSim.setPose(currentPose);
        RobotState.getInstance().addPose(Timer.getFPGATimestamp(), currentPose);
    }

    public void resetEstimator(Rotation2d angle, SwerveModulePosition[] modulePositions, Pose2d pose) {
        m_poseEstimator.resetPosition(angle, modulePositions, pose);
    }

    public void resetEstimator(Pose2d pose) {
        var data = RobotState.getInstance().getCurrentOdometryData().getSecond();
        resetEstimator(
                data.getGyroAngle(),
                data.getModulePositions(),
                pose
        );
    }

    private Pose2d getVisionPose() {
        if (visionPose == null)
            return new Pose2d();
        return visionPose;
    }
}
