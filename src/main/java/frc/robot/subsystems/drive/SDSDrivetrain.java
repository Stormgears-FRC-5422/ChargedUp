package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.ModuleConfiguration;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.drive.SwerveModules.SDSModule;
import frc.utils.stormSwerveLib.StormSwerveMk4ProtoHelper;

import java.util.List;

import static frc.robot.Constants.SwerveBotDriveConstants.*;

public class SDSDrivetrain extends DrivetrainBase {
    /**
     * The maximum voltage that will be delivered to the drive motors.
     * <p>
     * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
     */
    public static final double MAX_VOLTAGE = 12.0;
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
    public final double MAX_VELOCITY_METERS_PER_SECOND;
    /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
    public final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

    // TODO (Darren). From the WPI docs,
    //    The locations for the modules must be relative to the center of the robot.
    //    Positive x values represent moving toward the front of the robot whereas
    //    positive y values represent moving toward the left of the robot.
    // OK. So doesn't that mean the X value should be related to the length, not the width, of the robot?
    // of course that doesn't matter for a square robot...
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );

    private final SwerveDriveOdometry m_odometry;

    // TODO - The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
    // cause the angle reading to increase until it wraps back over to zero.
    private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

    // These are our modules. We initialize them in the constructor.
    private final SDSModule m_frontLeftModule;
    private final SDSModule m_frontRightModule;
    private final SDSModule m_backLeftModule;
    private final SDSModule m_backRightModule;
    private final SDSModule[] m_swerveModules;

    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    public SDSDrivetrain() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        StormSwerveMk4ProtoHelper.GearRatio theGearRatio;
        ModuleConfiguration moduleConfiguration;

        switch(kMK4iModuleKind) {
            case "L1":
                theGearRatio = StormSwerveMk4ProtoHelper.GearRatio.L1;
                moduleConfiguration = SdsModuleConfigurations.MK4I_L1;
                break;
            case "L2":
                theGearRatio = StormSwerveMk4ProtoHelper.GearRatio.L2;
                moduleConfiguration = SdsModuleConfigurations.MK4I_L2;
                break;
            case "L3":
            default:
                theGearRatio = StormSwerveMk4ProtoHelper.GearRatio.L3;  // Have to pick something
                moduleConfiguration = SdsModuleConfigurations.MK4I_L3;
        };

        // Derived from module details
        this.MAX_VELOCITY_METERS_PER_SECOND= kSparkMaxFreeSpeedRPM * kDriveSpeedScale / 60.0 *
                moduleConfiguration.getDriveReduction() *
                moduleConfiguration.getWheelDiameter() * Math.PI;
        this.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
                Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);


        m_frontLeftModule = new SDSModule(
                // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
                tab,
                // This can either be STANDARD or FAST depending on your gear configuration
                theGearRatio,
                // This is the ID of the drive motor
                FRONT_LEFT_MODULE_DRIVE_MOTOR,
                // This is the ID of the steer motor
                FRONT_LEFT_MODULE_STEER_MOTOR,
                // This is the ID of the steer encoder
                FRONT_LEFT_MODULE_STEER_ENCODER,
                // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
                FRONT_LEFT_MODULE_STEER_OFFSET
        );

        // We will do the same for the other modules
        m_frontRightModule = new SDSModule(
                tab,
                theGearRatio,
                FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                FRONT_RIGHT_MODULE_STEER_MOTOR,
                FRONT_RIGHT_MODULE_STEER_ENCODER,
                FRONT_RIGHT_MODULE_STEER_OFFSET
        );

        m_backLeftModule = new SDSModule(
                tab,
                theGearRatio,
                BACK_LEFT_MODULE_DRIVE_MOTOR,
                BACK_LEFT_MODULE_STEER_MOTOR,
                BACK_LEFT_MODULE_STEER_ENCODER,
                BACK_LEFT_MODULE_STEER_OFFSET
        );

        m_backRightModule = new SDSModule(
                tab,
                theGearRatio,
                BACK_RIGHT_MODULE_DRIVE_MOTOR,
                BACK_RIGHT_MODULE_STEER_MOTOR,
                BACK_RIGHT_MODULE_STEER_ENCODER,
                BACK_RIGHT_MODULE_STEER_OFFSET
        );

        m_swerveModules = new SDSModule[]{m_frontLeftModule, m_frontRightModule, m_backLeftModule, m_backRightModule};

        m_odometry = new SwerveDriveOdometry(m_kinematics, getGyroscopeRotation(), getPositions(), new Pose2d(new Translation2d(0d, 0d), new Rotation2d()));
    }

    private SwerveModulePosition[] getPositions() {
        return new SwerveModulePosition[] {
                m_frontLeftModule.getPosition(),
                m_frontRightModule.getPosition(),
                m_backLeftModule.getPosition(),
                m_backRightModule.getPosition()
        };
    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
     * 'forwards' direction.
     */
    public void zeroGyroscope() {
        m_navx.zeroYaw();
    }

    public Rotation2d getGyroscopeRotation() {
        if (m_navx.isMagnetometerCalibrated()) {
            // We will only get valid fused headings if the magnetometer is calibrated
            return Rotation2d.fromDegrees(m_navx.getFusedHeading());
        }

        // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
        return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
    }

    @Override
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    @Override
    public void setOdometry(Pose2d pose) {
        m_odometry.resetPosition(getGyroscopeRotation(), getPositions(), pose);
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        m_chassisSpeeds = chassisSpeeds;
    }

    @Override
    public void percentOutDrive(double tx, double ty, double rot) {
        
    }

    @Override
    public void periodic() {
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
        m_odometry.update(getGyroscopeRotation(), getPositions());

        m_frontLeftModule.setDesiredState(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle);
        m_frontRightModule.setDesiredState(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle);
        m_backLeftModule.setDesiredState(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle);
        m_backRightModule.setDesiredState(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle);
    }
}
