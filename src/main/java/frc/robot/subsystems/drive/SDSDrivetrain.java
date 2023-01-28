package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.NavX;

import static frc.robot.Constants.*;

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
    private final NavX m_navx = new NavX(); // NavX connected over MXP

    // These are our modules. We initialize them in the constructor.
    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;
    private final SwerveModule[] m_swerveModules;

    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    public SDSDrivetrain() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        Mk4iSwerveModuleHelper.GearRatio theGearRatio;
        ModuleConfiguration moduleConfiguration;

        switch(kMK4iModuleKind) {
            case "L1":
                theGearRatio = Mk4iSwerveModuleHelper.GearRatio.L1;
                moduleConfiguration = SdsModuleConfigurations.MK4I_L1;
                break;
            case "L2":
                theGearRatio = Mk4iSwerveModuleHelper.GearRatio.L2;
                moduleConfiguration = SdsModuleConfigurations.MK4I_L2;
                break;
            case "L3":
            default:
                theGearRatio = Mk4iSwerveModuleHelper.GearRatio.L3;  // Have to pick something
                moduleConfiguration = SdsModuleConfigurations.MK4I_L3;
        }

        // Derived from module details
        this.MAX_VELOCITY_METERS_PER_SECOND= kSparkMaxFreeSpeedRPM * kDriveSpeedScale / 60.0 *
                moduleConfiguration.getDriveReduction() *
                moduleConfiguration.getWheelDiameter() * Math.PI;
        this.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
                Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);


        m_frontLeftModule = Mk4iSwerveModuleHelper.createNeo(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
                theGearRatio,
                FRONT_LEFT_MODULE_DRIVE_MOTOR,
                FRONT_LEFT_MODULE_STEER_MOTOR,
                FRONT_LEFT_MODULE_STEER_ENCODER,
                FRONT_LEFT_MODULE_STEER_OFFSET
        );

        m_frontRightModule = Mk4iSwerveModuleHelper.createNeo(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(2, 0),
                theGearRatio,
                FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                FRONT_RIGHT_MODULE_STEER_MOTOR,
                FRONT_RIGHT_MODULE_STEER_ENCODER,
                FRONT_RIGHT_MODULE_STEER_OFFSET
        );

        m_backLeftModule = Mk4iSwerveModuleHelper.createNeo(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(4, 0),
                theGearRatio,
                BACK_LEFT_MODULE_DRIVE_MOTOR,
                BACK_LEFT_MODULE_STEER_MOTOR,
                BACK_LEFT_MODULE_STEER_ENCODER,
                BACK_LEFT_MODULE_STEER_OFFSET
        );

        m_backRightModule = Mk4iSwerveModuleHelper.createNeo(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(6, 0),
                theGearRatio,
                BACK_RIGHT_MODULE_DRIVE_MOTOR,
                BACK_RIGHT_MODULE_STEER_MOTOR,
                BACK_RIGHT_MODULE_STEER_ENCODER,
                BACK_RIGHT_MODULE_STEER_OFFSET
        );

        // We will do the same for the other modules
//        m_frontRightModule = new SDSModule(
//                tab,
//                theGearRatio,
//                FRONT_RIGHT_MODULE_DRIVE_MOTOR,
//                FRONT_RIGHT_MODULE_STEER_MOTOR,
//                FRONT_RIGHT_MODULE_STEER_ENCODER,
//                FRONT_RIGHT_MODULE_STEER_OFFSET
//        );
//
//        m_backLeftModule = new SDSModule(
//                tab,
//                theGearRatio,
//                BACK_LEFT_MODULE_DRIVE_MOTOR,
//                BACK_LEFT_MODULE_STEER_MOTOR,
//                BACK_LEFT_MODULE_STEER_ENCODER,
//                BACK_LEFT_MODULE_STEER_OFFSET
//        );
//
//        m_backRightModule = new SDSModule(
//                tab,
//                theGearRatio,
//                BACK_RIGHT_MODULE_DRIVE_MOTOR,
//                BACK_RIGHT_MODULE_STEER_MOTOR,
//                BACK_RIGHT_MODULE_STEER_ENCODER,
//                BACK_RIGHT_MODULE_STEER_OFFSET
//        );

        m_swerveModules = new SwerveModule[]{m_frontLeftModule, m_frontRightModule, m_backLeftModule, m_backRightModule};

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

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void setOdometry(Pose2d pose) {
        m_odometry.resetPosition(getGyroscopeRotation(), getPositions(), pose);
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        m_chassisSpeeds = chassisSpeeds;
    }

    @Override
    public void percentOutDrive(double tx, double ty, double rot) {
        double vxMetersPerSecond = tx * MAX_VELOCITY_METERS_PER_SECOND;
        double vyMetersPerSecond = ty * MAX_VELOCITY_METERS_PER_SECOND;
        double rotRadiansPerSecond = rot * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
        ChassisSpeeds speeds = new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, rotRadiansPerSecond);
        drive(speeds);
    }

    public void periodic() {
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
        m_odometry.update(getGyroscopeRotation(), getPositions());

        m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
        m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
        m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
        m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
    }
}
