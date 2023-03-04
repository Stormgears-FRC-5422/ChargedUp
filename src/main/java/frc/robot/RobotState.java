package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.constants.Constants;
import frc.robot.constants.ShuffleboardConstants;
import frc.utils.subsystemUtils.StormSubsystemBase;

import java.util.Map;
import java.util.TreeMap;

public class RobotState extends StormSubsystemBase {
    private static RobotState m_instance;
    private final Timer m_timer;

    private final TreeMap<Double, OdometryData> m_odometryData = new TreeMap<>();
    private final TreeMap<Double, Rotation2d> m_gyroData = new TreeMap<>();
    private final TreeMap<Double, Pose2d> m_visionData = new TreeMap<>();

    private Pose2d currentPose, startPose, lastPose;

    private final Field2d fieldSim;

    public static RobotState getInstance() {
        if (m_instance != null) return m_instance;

        m_instance = new RobotState();
        return m_instance;
    }

    private RobotState() {
        m_timer = new Timer();
        m_timer.stop();

        ShuffleboardTab tab = ShuffleboardConstants.getInstance().robotStateTab;
        ShuffleboardLayout layout = tab
                .getLayout("State", BuiltInLayouts.kList)
                .withPosition(0, 0)
                .withSize(2, 4);
        layout.addNumber("Pose X", () -> getCurrentPose().getX());
        layout.addNumber("Pose Y", () -> getCurrentPose().getY());
        layout.addNumber("Pose Angle", () -> getCurrentPose().getRotation().getDegrees());
        layout.addNumber("Robot Time", this::getTimeSeconds);
        layout.addNumber("Linear Velocity", this::getCurrentLinearVel);
        layout.addNumber("Rotational Velocity", this::getCurrentDegPerSecVel);
        fieldSim = new Field2d();
        tab.add(fieldSim).withWidget(BuiltInWidgets.kField)
                .withPosition(2, 0)
                .withSize(5, 4);
    }

    public double getTimeSeconds() {
        return m_timer.get();
    }

    public Pose2d getCurrentPose() {
        if (!Constants.usePoseEstimator) {
            System.out.println("NOT using pose estimator. Can't get current pose!");
            return getStartPose();
        }
        if (currentPose == null) {
            System.out.println("Using start pose for current pose: " + getStartPose());
            return getStartPose();
        }
        return currentPose;
    }

    public void setCurrentPose(Pose2d pose) {
        currentPose = pose;
    }

    public Pose2d getStartPose() {
        if (startPose == null) {
            System.out.println("Starting position was not set! Which is fine...");
            return new Pose2d();
        }
        return startPose;
    }

    public void setStartPose(Pose2d pose) {
        startPose = pose;
    }

    public Pose2d getLastPose() {
        if (!Constants.usePoseEstimator) {
            System.out.println("NOT using pose estimator. Can't get last pose!");
            return getCurrentPose();
        }
        if (lastPose == null) {
            System.out.println("Using current pose for last pose: " + getCurrentPose());
            return getCurrentPose();
        }
        return lastPose;
    }

    public void setLastPose(Pose2d lastPose) {
        this.lastPose = lastPose;
    }

    public void addGyroData(double time, Rotation2d angle) {
        m_gyroData.put(time, angle);
    }

    public Rotation2d getAngleAtTimeSeconds(double time) {
        var floorEntry = m_gyroData.floorEntry(time);
        var ceilEntry = m_gyroData.ceilingEntry(time);
        double timeFromFloor = time - floorEntry.getKey();
        if (floorEntry == null) {
            System.out.println("Don't have gyro data for time: " + time);
            return getStartPose().getRotation();
        }
        if (ceilEntry == null) {
            double rotationalVel = getCurrentDegPerSecVel();
            Rotation2d rotationFromFloor = Rotation2d.fromDegrees(rotationalVel * timeFromFloor);
            return floorEntry.getValue().plus(rotationFromFloor);
        }
        double timeFloorToCeiling = ceilEntry.getKey() - floorEntry.getKey();
        var deltaRotation = ceilEntry.getValue().minus(floorEntry.getValue());
        return floorEntry.getValue().plus(deltaRotation.times(timeFromFloor / timeFloorToCeiling));
    }

    public Rotation2d getCurrentGyroRotation() {
        if (!Constants.useNavX) {
            System.out.println("NOT using gyro. Can't get current gyro rotation!");
            return Rotation2d.fromDegrees(0);
        }
        return m_gyroData.lastEntry().getValue();
    }

    public void addOdometryData(double time, OdometryData odometryData) {
        m_odometryData.put(time, odometryData);
    }

    public Map.Entry<Double, OdometryData> getLatestOdometryData() {
        return m_odometryData.lastEntry();
    }

    /** Must provide own timstamp with this function as camera will have delay*/
    public void addVisionData(double timeStampSeconds, Pose2d visionData) {
        m_visionData.put(timeStampSeconds, visionData);
    }

    public Map.Entry<Double, Pose2d> getLatestVisionData() {
        return m_visionData.lastEntry();
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
        return Math.abs(getCurrentPose().getRotation().minus(getLastPose().getRotation()).getDegrees());
    }

    public double getCurrentDegPerSecVel() {
        return getDeltaDegrees() / 0.02;
    }

    public void enabledInit() {
        m_timer.reset();
        m_timer.start();

        m_odometryData.clear();
        m_gyroData.clear();
        m_visionData.clear();

        currentPose = null;
        lastPose = null;
    }

    public void disabledInit() {
        m_timer.stop();
    }

    public void lastPeriodic() {
        fieldSim.setRobotPose(getCurrentPose());

        double currentTime = Timer.getFPGATimestamp();
        m_odometryData.tailMap(currentTime - 1.0, true);
        m_visionData.tailMap(currentTime - 1.0, true);
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
