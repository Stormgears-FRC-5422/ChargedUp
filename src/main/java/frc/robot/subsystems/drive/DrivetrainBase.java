package frc.robot.subsystems.drive;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.NavX;

import static frc.robot.Constants.*;

public abstract class DrivetrainBase extends SubsystemBase {
    // TODO - The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
    // cause the angle reading to increase until it wraps back over to zero.
    protected NavX m_gyro = new NavX();

    // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
    //  The formula for calculating the theoretical maximum velocity is:
    //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
    //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
    //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
    //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
    /**
     * The maximum velocity of the robot in meters per second.
     * <p>
     * This is a measure of how fast the robot should be able to drive in a straight line.
     */
    public double m_maxVelocityMetersPerSecond;
    /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
    public double m_maxAngularVelocityRadiansPerSecond;

    // The chassis speeds will always be scaled by this factor. It defaults to kDriveSpeedScale, but can be reset
    // (say by using the slider on the joystick)
    protected double m_driveSpeedScale = 0;

    protected ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    protected ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    DrivetrainBase() {
        setDriveSpeedScale(kDriveSpeedScale);
    }

    protected void setMaxVelocities(double maxVelocityMetersPerSecond, double maxAngularVelocityRadiansPerSecond) {
        this.m_maxVelocityMetersPerSecond = maxVelocityMetersPerSecond;
        this.m_maxAngularVelocityRadiansPerSecond = maxAngularVelocityRadiansPerSecond;
    }

    public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
        // Scale incoming speeds
        ChassisSpeeds s = new ChassisSpeeds(
                m_driveSpeedScale * speeds.vxMetersPerSecond,
                m_driveSpeedScale * speeds.vyMetersPerSecond,
                m_driveSpeedScale * speeds.omegaRadiansPerSecond);

        if (fieldRelative) {
            m_chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(s, getGyroscopeRotation());
        }
        else {
            m_chassisSpeeds = s;
        }
    }

    public void percentOutDrive(ChassisSpeeds speeds, boolean fieldRelative) {
        drive(new ChassisSpeeds(speeds.vxMetersPerSecond * m_maxVelocityMetersPerSecond,
                        speeds.vyMetersPerSecond * m_maxVelocityMetersPerSecond,
                        speeds.omegaRadiansPerSecond * m_maxAngularVelocityRadiansPerSecond),
                fieldRelative);
    }

    public void setDriveSpeedScale(double scale) {
        m_driveSpeedScale = MathUtil.clamp(scale, 0, kDriveSpeedScale);
    }

    public void stopDrive() {
        drive(new ChassisSpeeds(0, 0, 0), false);
    }

    public void zeroGyroscope() {
        if (m_gyro == null) return;
        m_gyro.zeroYaw();
    }
    public Rotation2d getGyroscopeRotation() {
        if (m_gyro == null) return Rotation2d.fromDegrees(0);
//
//        if (m_gyro.isMagnetometerCalibrated()) {
//            // We will only get valid fused headings if the magnetometer is calibrated
//            System.out.println("Current heading: " + m_gyro.getFusedHeading());
//            return Rotation2d.fromDegrees(m_gyro.getFusedHeading());
//        }

        // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
        return Rotation2d.fromDegrees(360.0 - m_gyro.getYaw());
    }

    public ChassisSpeeds getCurrentChassisSpeeds() {
        return m_chassisSpeeds;
    }

    public abstract SwerveDriveKinematics getSwerveDriveKinematics();
    public abstract SwerveModulePosition[] getSwerveModulePositions();
    public abstract void goToTrajectoryState(Trajectory.State goalState);
    public abstract void goToPPTrajectoryState(PathPlannerTrajectory.PathPlannerState goalState);
    public abstract void onEnable();
    public abstract void onDisable();
}
