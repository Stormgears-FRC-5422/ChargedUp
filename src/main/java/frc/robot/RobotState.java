package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Map;
import java.util.TreeMap;

public class RobotState {
    private static RobotState m_instance;
    private final Timer m_timer;

    private TreeMap<Double, DriveData> m_driveDataSet = new TreeMap<>();
    private TreeMap<Double, Pose2d> m_visionDataSet = new TreeMap<>();

    private Pose2d m_currentPose;
    private Pose2d m_startPose;
    private Pose2d m_lastPose;

    private Rotation2d m_currentGyroRotation;

    private final ShuffleboardTab mainTab;

    private final Field2d field2d;
    public static RobotState getInstance() {
        if (m_instance != null) return m_instance;

        m_instance = new RobotState();
        return m_instance;
    }

    private RobotState() {
        m_timer = new Timer();
        m_timer.stop();

        mainTab = Shuffleboard.getTab("MainTab");
        ShuffleboardLayout layout = mainTab.getLayout("State", BuiltInLayouts.kGrid)
                .withPosition(0, 0)
                .withSize(4, 4);
        layout.addNumber("pose x", () -> getCurrentPose().getX());
        layout.addNumber("pose y", () -> getCurrentPose().getY());
        layout.addNumber("pose angle", () -> getCurrentPose().getRotation().getDegrees());
        layout.addNumber("time", this::getTimeSeconds);
        field2d = new Field2d();
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
        m_currentPose = pose;
    }

    public void resetPose() {
        resetPose(new Pose2d());
    }

    public void setCurrentGyroRotation(Rotation2d rotation) {
        this.m_currentGyroRotation = rotation;
    }

    public Rotation2d getCurrentGyroRotation() {
        return this.m_currentGyroRotation;
    }

    public Pose2d getCurrentPose() {
        if (m_currentPose == null) return getStartPose();
        return m_currentPose;
    }

    public void setCurrentPose(Pose2d pose) {
        m_currentPose = pose;
    }

    public Pose2d getStartPose() {
        if (m_startPose == null) {
            System.out.println("********* Starting position was not set! *********");
            return new Pose2d();
        }
        return m_startPose;
    }

    public void setStartPose(Pose2d pose) {
        m_startPose = pose;
    }

    public Pose2d getLastPose() {
        if (m_lastPose == null) return getCurrentPose();
        return m_lastPose;
    }

    public void setLastPose(Pose2d m_lastPose) {
        this.m_lastPose = m_lastPose;
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

    public double getCurrentXVel() {
        if (m_lastPose == null) return 0;
        return ((getCurrentPose().getX() - getLastPose().getX())/20.) * 1000;
    }

    public double getCurrentYVel() {
        if (m_lastPose == null) return 0;
        return ((getCurrentPose().getY() - getLastPose().getY())/20.) * 1000;
    }

    public double getCurrentOmegaVel() {
        if (m_lastPose == null) return 0;
        return ((getCurrentPose().getRotation().getDegrees() - getLastPose().getRotation().getDegrees())/20.) * 1000;
    }

    public double getDeltaDistance() {
        if (m_lastPose == null) return 0;
        Translation2d m_lastTranslation = getLastPose().getTranslation();
        Translation2d m_currentTranslation = getCurrentPose().getTranslation();
        return m_currentTranslation.getDistance(m_lastTranslation);
    }

    public double getCurrentLinearVel() {
        //delta distance / delta time
        return getDeltaDistance()/0.02;
    }

    public void onEnable() {
        startTimer();
        m_driveDataSet = new TreeMap<Double, DriveData>();
    }

    public void onDisable() {
        stopTimer();
        resetPose();
    }

    public void update() {
        field2d.setRobotPose(getCurrentPose());
        SmartDashboard.putData(field2d);

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
