package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.subsystems.IEnabledDisabled;

import java.util.Map;
import java.util.TreeMap;

public class RobotState implements IEnabledDisabled {
    private static RobotState m_instance;
    private final Timer m_timer;

    private TreeMap<Double, DriveData> m_driveDataSet = new TreeMap<>();
    private TreeMap<Double, Pose2d> m_visionDataSet = new TreeMap<>();

    private Pose2d currentPose, startPose, lastPose;
    private Rotation2d currentRotation, lastRotation;

    private final ShuffleboardTab tab;
    private Field2d field2d;

    public static RobotState getInstance() {
        if (m_instance != null) return m_instance;

        m_instance = new RobotState();
        return m_instance;
    }

    private RobotState() {
        m_timer = new Timer();
        m_timer.stop();

        tab = Shuffleboard.getTab("Robot State");
        ShuffleboardLayout layout = tab
                .getLayout("State", BuiltInLayouts.kList)
                .withPosition(0, 0)
                .withSize(2, 4);
        layout.addNumber("pose x", () -> getCurrentPose().getX());
        layout.addNumber("pose y", () -> getCurrentPose().getY());
        layout.addNumber("pose angle", () -> getCurrentPose().getRotation().getDegrees());
        layout.addNumber("time", this::getTimeSeconds);
        layout.addNumber("linear velocity", this::getCurrentLinearVel);
        layout.addNumber("rotational velocity", this::getCurrentRotationalVel);
        field2d = new Field2d();
        tab.add(field2d).withWidget(BuiltInWidgets.kField)
                .withPosition(2, 0)
                .withSize(4, 3);
    }

    public void startTimer() {
        m_timer.reset();
        m_timer.start();
    }

    public void stopTimer() {
        m_timer.stop();
        m_timer.reset();
    }

    public double getTimeSeconds() {
        return m_timer.get();
    }

    public void resetPose(Pose2d pose) {
        currentPose = pose;
    }

    public void resetPose() {
        resetPose(new Pose2d());
    }

    public Pose2d getCurrentPose() {
        if (currentPose == null) return getStartPose();
        return currentPose;
    }

    public void setCurrentPose(Pose2d pose) {
        currentPose = pose;
    }

    public Pose2d getStartPose() {
        if (startPose == null) {
            System.out.println("********* Starting position was not set! *********");
            return new Pose2d();
        }
        return startPose;
    }

    public void setStartPose(Pose2d pose) {
        startPose = pose;
    }

    public Pose2d getLastPose() {
        if (lastPose == null) return getCurrentPose();
        return lastPose;
    }

    public void setLastPose(Pose2d lastPose) {
        this.lastPose = lastPose;
    }

    public Rotation2d getCurrentGyroRotation() {
        if (currentRotation == null) return Rotation2d.fromDegrees(0);
        return currentRotation;
    }

    public void setCurrentGyroRotation(Rotation2d currentRotation) {
        this.currentRotation = currentRotation;
    }

    public Rotation2d getLastGyroRotation() {
        if (lastRotation == null) return getCurrentGyroRotation();
        return lastRotation;
    }

    public void setLastGyroRotation(Rotation2d lastRotation) {
        this.lastRotation = lastRotation;
    }

    public void addDriveData(DriveData driveData) {
        if (m_timer == null) {
            System.out.println("Timer was not set!!!!!!!");
            return;
        }
        m_driveDataSet.put(Timer.getFPGATimestamp() * 1000., driveData);
    }

    public Map.Entry<Double, DriveData> getLatestDriveData() {
        return m_driveDataSet.lastEntry();
    }

    /**
     * Must provide own timstamp with this function as camera will have delay
     * @param timeStamp
     */
    public void addVisionData(double timeStamp, Pose2d visionData) {
        m_visionDataSet.put(timeStamp, visionData);
    }

    public Map.Entry<Double, Pose2d> getLatestVisionData() {
        return m_visionDataSet.lastEntry();
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
        return Math.abs(getCurrentGyroRotation().getDegrees() - getLastGyroRotation().getDegrees());
    }

    public double getCurrentRotationalVel() {
        return getDeltaDegrees() / 0.02;
    }

    public void onEnable() {
        startTimer();
        m_driveDataSet = new TreeMap<Double, DriveData>();
        m_visionDataSet = new TreeMap<Double, Pose2d>();
    }

    public void onDisable() {
        stopTimer();
        resetPose();
    }

    public void update() {
        field2d.setRobotPose(getCurrentPose());

        double currentTimeMs = Timer.getFPGATimestamp();
        m_driveDataSet.tailMap(currentTimeMs - 2000, true);
        m_visionDataSet.tailMap(currentTimeMs - 2000, true);
    }

    public static class DriveData {
        private SwerveModulePosition[] modulePositions;
        private Rotation2d gyroAngle;

        public DriveData(SwerveModulePosition[] modulePositions, Rotation2d gyroAngle) {
            this.modulePositions = modulePositions;
            this.gyroAngle = gyroAngle;
        }

        public DriveData() {
            this.modulePositions = new SwerveModulePosition[] {
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
            };
            this.gyroAngle = new Rotation2d();
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
