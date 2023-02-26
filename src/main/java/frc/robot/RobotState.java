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
    private Rotation2d currentGyroRotation, lastGyroRotation;

    private final Field2d fieldSim;

    public static RobotState getInstance() {
        if (m_instance != null) return m_instance;

        m_instance = new RobotState();
        return m_instance;
    }

    private RobotState() {
        m_timer = new Timer();
        m_timer.stop();

        ShuffleboardTab tab = Shuffleboard.getTab("Robot State");
        ShuffleboardLayout layout = tab
                .getLayout("State", BuiltInLayouts.kList)
                .withPosition(0, 0)
                .withSize(2, 4);
        layout.addNumber("Pose X", () -> getCurrentPose().getX());
        layout.addNumber("Pose Y", () -> getCurrentPose().getY());
        layout.addNumber("Pose Angle", () -> getCurrentPose().getRotation().getDegrees());
        layout.addNumber("Robot Time", this::getTimeSeconds);
        layout.addNumber("Linear Velocity", this::getCurrentLinearVel);
        layout.addNumber("Rotational Velocity", this::getCurrentRotationalVel);
        fieldSim = new Field2d();
        tab.add(fieldSim).withWidget(BuiltInWidgets.kField)
                .withPosition(2, 0)
                .withSize(4, 3);
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
        if (currentGyroRotation == null) return Rotation2d.fromDegrees(0);
        return currentGyroRotation;
    }

    public void setCurrentGyroRotation(Rotation2d currentRotation) {
        this.currentGyroRotation = currentRotation;
    }

    public Rotation2d getLastGyroRotation() {
        if (lastGyroRotation == null) return getCurrentGyroRotation();
        return lastGyroRotation;
    }

    public void setLastGyroRotation(Rotation2d lastRotation) {
        this.lastGyroRotation = lastRotation;
    }

    public void addDriveData(DriveData driveData) {
        if (m_timer == null) {
            System.out.println("Timer was not set!!!!!!!");
            return;
        }
        m_driveDataSet.put(Timer.getFPGATimestamp(), driveData);
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
        m_timer.reset();
        m_timer.start();
        m_driveDataSet = new TreeMap<Double, DriveData>();
        m_visionDataSet = new TreeMap<Double, Pose2d>();
    }

    public void onDisable() {
        m_timer.stop();
        resetPose();
    }

    public void update() {
        fieldSim.setRobotPose(getCurrentPose());

        double currentTimeMs = Timer.getFPGATimestamp();
        m_driveDataSet.tailMap(currentTimeMs - 2000, true);
        m_visionDataSet.tailMap(currentTimeMs - 2000, true);
    }

    public static class DriveData {
        private final SwerveModulePosition[] modulePositions;
        private final Rotation2d gyroAngle;

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
