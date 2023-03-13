package frc.robot;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import frc.robot.constants.Constants;
import frc.robot.constants.ShuffleboardConstants;
import frc.robot.subsystems.vision.Vision;
import frc.utils.subsystemUtils.StormSubsystemBase;

import java.util.TreeMap;
import java.util.Vector;

public class RobotState extends StormSubsystemBase {
    private static RobotState m_instance;
    private final Timer m_timer;


    private Pair<Double, OdometryData> currentOdometryData = null;
    private Pair<Double, Vector<Vision.AprilTagData>> currentVisionData = null;
    private int visionLogCounter = 0;
    private TreeMap<Double, Rotation2d> gyroData = new TreeMap<>();

    private Pose2d currentPose, startPose, lastPose;

    private GenericEntry xStartEntry, yStartEntry, rotStartEntry;

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
        layout.addNumber("Rotational Velocity", this::getCurrentDegPerSecVel);

        var startPoseSetter = ShuffleboardConstants.getInstance().driverTab
                .getLayout("Set Start Pose", BuiltInLayouts.kGrid)
                .withPosition(0, 3).withSize(3, 1);

        xStartEntry = startPoseSetter
                .add("X", getStartPose().getX())
                .withPosition(0, 0).getEntry();

        yStartEntry = startPoseSetter
                .add("Y", getStartPose().getY())
                .withPosition(1, 0).getEntry();

        rotStartEntry = startPoseSetter
                .add("Rot", getStartPose().getRotation().getDegrees())
                .withPosition(2, 0).getEntry();
    }

    public double getTimeSeconds() {
        return m_timer.get();
    }

    public Pose2d getCurrentPose() {
        if (!Constants.Toggles.usePoseEstimator) {
//            System.out.println("NOT using pose estimator. Can't get current pose!");
            return getStartPose();
        }
        if (currentPose == null) {
//            System.out.println("Using start pose for current pose: " + getStartPose());
            return getStartPose();
        }
        return currentPose;
    }

    public void setCurrentPose(Pose2d pose) {
        currentPose = pose;
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

    public Pose2d getLastPose() {
        if (!Constants.Toggles.usePoseEstimator) {
//            System.out.println("NOT using pose estimator. Can't get last pose!");
            return getCurrentPose();
        }
        if (lastPose == null) {
//            System.out.println("Using current pose for last pose: " + getCurrentPose());
            return getCurrentPose();
        }
        return lastPose;
    }

    public void setLastPose(Pose2d lastPose) {
        this.lastPose = lastPose;
    }

    public void addGyroData(double time, Rotation2d angle) {
        gyroData.put(time, angle);
    }

    public Rotation2d getAngleAtTimeSeconds(double time) {
        var floorEntry = gyroData.floorEntry(time);
        var ceilEntry = gyroData.ceilingEntry(time);
        if (floorEntry == null) {
            if (ceilEntry != null) return ceilEntry.getValue();
            return getCurrentGyroRotation();
        }
        double timeFromFloor = time - floorEntry.getKey();
        if (ceilEntry == null) {
            double rotationalVel = getCurrentDegPerSecVel();
            Rotation2d rotationFromFloor = Rotation2d.fromDegrees(rotationalVel * timeFromFloor);
            return floorEntry.getValue().plus(rotationFromFloor);
        }
        if (Math.abs(ceilEntry.getValue().getDegrees() - floorEntry.getValue().getDegrees()) >= 50) {
            return ceilEntry.getValue();
        }
        double timeFloorToCeiling = ceilEntry.getKey() - floorEntry.getKey();
        return floorEntry.getValue().interpolate(ceilEntry.getValue(), timeFromFloor / timeFloorToCeiling);
    }

    public Rotation2d getCurrentGyroRotation() {
        if (!Constants.Toggles.useNavX) {
            System.out.println("NOT using gyro. Can't get current gyro rotation!");
            return Rotation2d.fromDegrees(0);
        }
        var entry = gyroData.lastEntry();
        return (entry != null)? entry.getValue() : new Rotation2d();
    }

    public void setOdometryData(double time, OdometryData odometryData) {
        currentOdometryData = new Pair<>(time, odometryData);
    }

    public Pair<Double, OdometryData> getCurrentOdometryData() {
        return currentOdometryData;
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

    public double getDeltaDistanceMeters() {
        Translation2d m_lastTranslation = getLastPose().getTranslation();
        Translation2d m_currentTranslation = getCurrentPose().getTranslation();
        return m_currentTranslation.getDistance(m_lastTranslation);
    }

    public double getCurrentLinearVel() {
        return getDeltaDistanceMeters() / 0.02;
    }

    public double getDeltaDegrees() {
        return getLastPose().getRotation().minus(getCurrentPose().getRotation()).getDegrees();
    }

    public double getCurrentDegPerSecVel() {
        return getDeltaDegrees() / 0.02;
    }

    public void enabledInit() {
        m_timer.reset();
        m_timer.start();

        currentOdometryData = null;
        currentVisionData = null;
        gyroData = new TreeMap<>();

        currentPose = null;
        lastPose = null;

        setStartPose(
                new Pose2d(
                    xStartEntry.getDouble(0.0),
                    yStartEntry.getDouble(0.0),
                    Rotation2d.fromDegrees(rotStartEntry.getDouble(0.0)))
        );
    }

    public void disabledInit() {
        m_timer.stop();

        currentOdometryData = null;
        currentVisionData = null;
        gyroData = new TreeMap<>();

        currentPose = null;
        lastPose = null;
    }

    public static class OdometryData {
        private final SwerveModulePosition[] modulePositions;
        private final Rotation2d gyroAngle;

        public OdometryData(SwerveModulePosition[] modulePositions, Rotation2d gyroAngle) {
            this.modulePositions = modulePositions;
            this.gyroAngle = gyroAngle;
        }

        public Rotation2d getGyroAngle() {
            return gyroAngle;
        }

        public SwerveModulePosition[] getModulePositions() {
            return modulePositions;
        }
    }
    public static class ArmData {

    }
}
