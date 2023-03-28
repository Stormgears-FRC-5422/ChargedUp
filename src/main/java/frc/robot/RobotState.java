package frc.robot;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import frc.robot.constants.Constants;
import frc.robot.constants.ShuffleboardConstants;
import frc.robot.subsystems.vision.Vision;
import frc.utils.subsystemUtils.StormSubsystemBase;

import java.util.Arrays;
import java.util.Map;
import java.util.Vector;

public class RobotState extends StormSubsystemBase {
    private static RobotState m_instance;
    private final Timer m_timer;


    private Pair<Double, OdometryData> currentOdometryData = null;
    private Pair<Double, Vector<Vision.AprilTagData>> currentVisionData = null;
    private Pair<Double, Rotation2d> currentGyroData = null;
    private int visionLogCounter = 0;

    private final TimeInterpolatableBuffer<Pose2d> poseRecord =
            TimeInterpolatableBuffer.createBuffer(0.5);
    private Map.Entry<Double, Pose2d> currentEntry, lastEntry;

    private Pose2d startPose;
    private DriverStation.Alliance currentAlliance = DriverStation.Alliance.Blue;
    private Constants.LidarRange lidarRange = Constants.LidarRange.CONE;

    private GenericEntry xEntry, yEntry, rotEntry;

    public static RobotState getInstance() {
        if (m_instance != null) return m_instance;

        m_instance = new RobotState();
        return m_instance;
    }

    private RobotState() {
        m_timer = new Timer();
        m_timer.stop();

        var layout = ShuffleboardConstants.getInstance().robotStateList;
        layout.addNumber("Pose X", () -> getCurrentPose().getX());
        layout.addNumber("Pose Y", () -> getCurrentPose().getY());
        layout.addNumber("Pose Angle", () -> getCurrentPose().getRotation().getDegrees());
        layout.addNumber("Robot Time", this::getTimeSeconds);
        layout.addNumber("Linear Velocity", this::getCurrentLinearVel);
        layout.addNumber("Rotational Velocity", this::getCurrentRotVelDeg);

        var poseGridLayout = ShuffleboardConstants.getInstance().driverTab
                .getLayout("Set Start Pose", BuiltInLayouts.kGrid)
                .withProperties(Map.of("number of columns", 3, "number of rows", 1))
                .withPosition(0, 3).withSize(2, 1);

        xEntry = poseGridLayout
                .add("X", getStartPose().getX())
                .withPosition(0, 0).getEntry();

        yEntry = poseGridLayout
                .add("Y", getStartPose().getY())
                .withPosition(1, 0).getEntry();

        rotEntry = poseGridLayout
                .add("Rot", getStartPose().getRotation().getDegrees())
                .withPosition(2, 0).getEntry();

        poseRecord.clear();

        currentOdometryData = null;
        currentGyroData = null;
        currentVisionData = null;
    }

    public void setCurrentAlliance(DriverStation.Alliance alliance) {
        System.out.println("Alliance set to: " + alliance);
        currentAlliance = alliance;
    }

    public DriverStation.Alliance getCurrentAlliance() {
//        System.out.println(currentAlliance);
        return currentAlliance;
    }

    public double getTimeSeconds() {
        return m_timer.get();
    }

    public void addPose(double time, Pose2d pose) {
        poseRecord.addSample(time, pose);
        lastEntry = currentEntry;
        currentEntry = poseRecord.getInternalBuffer().lastEntry();
    }

    public Pose2d getCurrentPose() {
        if (!Constants.Toggles.usePoseEstimator ||
                currentEntry == null) {
//            System.out.println("NOT using pose estimator. Can't get current pose!");
            return getStartPose();
        }
        return currentEntry.getValue();
    }

    public Pose2d getLastPose() {
        if (!Constants.Toggles.usePoseEstimator || lastEntry == null)
            return getCurrentPose();
        return lastEntry.getValue();
    }

    public Pose2d getStartPose() {
        if (startPose == null) {
//            System.out.println("Starting position was not set! Which is fine...");
            return new Pose2d();
        }
        return startPose;
    }

    public void setStartPose(Pose2d pose) {
        startPose = pose;
    }

    public Pose2d getPoseAtTime(double time) {
        if (poseRecord.getSample(time).isPresent()) {
            return poseRecord.getSample(time).get();
        }
        return getCurrentPose();
    }

    /** returns rotation at that time in seconds */
    public Rotation2d getRotationAtTime(double time) {
        return getPoseAtTime(time).getRotation();
    }

    public Translation2d getTranslationAtTime(double time) {
        return getPoseAtTime(time).getTranslation();
    }

    static double d = 0.02;
    public double getLinearVelAtTime(double time) {
        Translation2d translation = getTranslationAtTime(time);
        Translation2d prevTranslation = getTranslationAtTime(time - d);
        return (translation.getDistance(prevTranslation) / d) + 0.000001;
    }

    public double getRotationalVelAtTime(double time) {
        Rotation2d rotation = getRotationAtTime(time);
        Rotation2d prevRotation = getRotationAtTime(time - d);
        return (Math.abs(rotation.getDegrees() - prevRotation.getDegrees()) / d) + 0.000001;
    }

    public void setOdometryData(double time, OdometryData odometryData) {
        currentOdometryData = new Pair<>(time, odometryData);
    }

    public Pair<Double, OdometryData> getCurrentOdometryData() {
        return (currentOdometryData != null)? currentOdometryData : new Pair<>(0.0, new OdometryData());
    }

    public void setGyroData(double time, Rotation2d angle) {
        currentGyroData = new Pair<>(time, angle);
    }

    public Rotation2d getCurrentGyroData() {
        if (!Constants.Toggles.useNavX) {
            System.out.println("NOT using gyro. Can't get current gyro rotation!");
            return new Rotation2d();
        }
        return (currentGyroData != null)? currentGyroData.getSecond() : new Rotation2d();
    }

    public void setVisionData(double time, Vector<Vision.AprilTagData> visionData) {
        if (++visionLogCounter % 25 == 0) {
//            System.out.println("Robot State vision data \n timestamp: " + time);
//            for (var tag : visionData) {
//                System.out.println(tag);
//            }
        }
        currentVisionData = new Pair<>(time, visionData);
    }

    public Pair<Double, Vector<Vision.AprilTagData>> getCurrentVisionData() {
        return currentVisionData;
    }

    public Pair<Double, Twist2d> getDeltaPoseWithTime() {
        if (currentEntry == null || lastEntry == null)
            return new Pair<> (0.02, new Twist2d(0, 0, 0));
        return new Pair<>(currentEntry.getKey() - lastEntry.getKey(), lastEntry.getValue().log(currentEntry.getValue()));
    }

    public double getDeltaDistanceMeters() {
        Translation2d m_lastTranslation = getLastPose().getTranslation();
        Translation2d m_currentTranslation = getCurrentPose().getTranslation();
        return m_currentTranslation.getDistance(m_lastTranslation);
    }

    public double getCurrentLinearVel() {
        var lastEntry = poseRecord.getInternalBuffer().lastEntry();
        if (lastEntry == null)
            return 0;
        var lowerEntry = poseRecord.getInternalBuffer()
                .lowerEntry(poseRecord.getInternalBuffer().lastKey());
        if (lowerEntry == null)
            return 0;
        return lastEntry.getValue().getTranslation().getDistance(lowerEntry.getValue().getTranslation()) /
                (lastEntry.getKey() - lowerEntry.getKey());
    }

    public double getDeltaDegrees() {
        return getLastPose().getRotation().minus(getCurrentPose().getRotation()).getDegrees();
    }

    public double getCurrentRotVelDeg() {
        var twist = getDeltaPoseWithTime();
        return Math.toRadians(twist.getSecond().dtheta) / twist.getFirst();
    }

    public double getCurrentXVel() {
        var twist = getDeltaPoseWithTime();
        return twist.getSecond().dx / twist.getFirst();
    }

    public double getCurrentYVel() {
        var twist = getDeltaPoseWithTime();
        return twist.getSecond().dy / twist.getFirst();
    }

    public void setLidarRange(Constants.LidarRange type) {
        lidarRange = type;
    }

    public Constants.LidarRange getLidarRange() {
        return lidarRange;
    }

    @Override
    public void lastPeriodic() {
        // clear map
//        while (poseMap.size() > 5 &&
//                poseMap.firstKey() < Timer.getFPGATimestamp() - 0.5) {
//            poseMap.pollFirstEntry();
//        }
        Pose2d currentPose = getCurrentPose();
        xEntry.setDouble(currentPose.getX());
        yEntry.setDouble(currentPose.getY());
        rotEntry.setDouble(currentPose.getRotation().getDegrees());
    }

    public void autoInit() {
//        System.out.println("Enabled init in robotState!");
        m_timer.reset();
        m_timer.start();

        currentOdometryData = null;
        currentVisionData = null;
        currentGyroData = null;

        poseRecord.clear();

//        setStartPose(
//                new Pose2d(
//                    xStartEntry.getDouble(0.0),
//                    yStartEntry.getDouble(0.0),
//                    Rotation2d.fromDegrees(rotStartEntry.getDouble(0.0)))
//        );
    }

    public void disabledInit() {
//        m_timer.stop();
//
//        currentOdometryData = null;
//        currentVisionData = null;
//        currentGyroData = null;
//
//        poseRecord.clear();
    }

    public static class OdometryData {
        private final SwerveModulePosition[] modulePositions;
        private final Rotation2d gyroAngle;

        public OdometryData(SwerveModulePosition[] modulePositions, Rotation2d gyroAngle) {
            this.modulePositions = modulePositions;
            this.gyroAngle = gyroAngle;
        }

        public OdometryData() {
            this(new SwerveModulePosition[]{
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
            }, new Rotation2d());
        }

        public Rotation2d getGyroAngle() {
            return gyroAngle;
        }

        public SwerveModulePosition[] getModulePositions() {
            return modulePositions;
        }

        @Override
        public String toString() {
            return "OdometryData{" +
                    "modulePositions=" + Arrays.toString(modulePositions) +
                    ", gyroAngle=" + gyroAngle +
                    '}';
        }
    }
    public static class ArmData {

    }
}
