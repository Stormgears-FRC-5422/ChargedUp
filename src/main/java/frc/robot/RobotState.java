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
        layout.addNumber("Rotational Velocity", this::getCurrentRotationalVel);
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

    public Rotation2d getCurrentGyroRotation() {
        if (!Constants.useNavX) {
            System.out.println("NOT using gyro. Can't get current gyro rotation!");
            return Rotation2d.fromDegrees(0);
        }
        if (currentGyroRotation == null) {
            System.out.println("Current gyro rotation not set!");
            return Rotation2d.fromDegrees(0);
        }
        return currentGyroRotation;
    }

    public void setCurrentGyroRotation(Rotation2d currentRotation) {
        this.currentGyroRotation = currentRotation;
    }

    public Rotation2d getLastGyroRotation() {
        if (!Constants.useNavX) {
            System.out.println("NOT using gyro. Can't get current gyro rotation!");
            return Rotation2d.fromDegrees(0);
        }
        if (lastGyroRotation == null) {
            System.out.println("Last gyro rotation not set!");
            return getCurrentGyroRotation();
        }
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

    /** Must provide own timstamp with this function as camera will have delay*/
    public void addVisionData(double timeStampSeconds, Pose2d visionData) {
        m_visionDataSet.put(timeStampSeconds, visionData);
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

    public void enabledInit() {
        m_timer.reset();
        m_timer.start();
        m_driveDataSet.clear();
        m_visionDataSet.clear();

        currentPose = null;
        lastPose = null;
        currentGyroRotation = null;
        lastGyroRotation = null;
    }

    public void disabledInit() {
        m_timer.stop();
    }

    public void lastPeriodic() {
        fieldSim.setRobotPose(getCurrentPose());

        double currentTime = Timer.getFPGATimestamp();
        m_driveDataSet.tailMap(currentTime - 1.0, true);
        m_visionDataSet.tailMap(currentTime - 1.0, true);
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
