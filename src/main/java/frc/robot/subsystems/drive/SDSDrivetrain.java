package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.CANCoder;
import com.swervedrivespecialties.swervelib.*;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.*;

public class SDSDrivetrain extends DrivetrainBase {
    /**
     * The maximum voltage that will be delivered to the drive motors.
     * <p>
     * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
     */
    public static final double MAX_VOLTAGE = 12.0;
    // TODO (Darren). From the WPI docs,
    //    The locations for the modules must be relative to the center of the robot.
    //    Positive x values represent moving toward the front of the robot whereas
    //    positive y values represent moving toward the left of the robot.
    // OK. So doesn't that mean the X value should be related to the length, not the width, of the robot?
    // of course that doesn't matter for a square robot...
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(DRIVETRAIN_WHEELBASE_METERS / 2.0, DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
            // Front right
            new Translation2d(DRIVETRAIN_WHEELBASE_METERS / 2.0, -DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
            // Back left
            new Translation2d(-DRIVETRAIN_WHEELBASE_METERS / 2.0, DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
            // Back right
            new Translation2d(-DRIVETRAIN_WHEELBASE_METERS / 2.0, -DRIVETRAIN_TRACKWIDTH_METERS / 2.0)
    );

    private final SwerveDrivePoseEstimator m_poseEstimator;

    private CANCoder fl;
    private CANCoder fr;
    private CANCoder bl;
    private CANCoder br;

    // These are our modules. We initialize them in the constructor.
    // TODO use StormSparks with voltage and current safeties
    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;

    private final Field2d field = new Field2d();

    public SDSDrivetrain() {
        initEncoders();

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        Mk4iSwerveModuleHelper.GearRatio theGearRatio;
        ModuleConfiguration moduleConfiguration;
        Mk4ModuleConfiguration modifiedModuleConfiguration;

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

//        modifiedModuleConfiguration = new Mk4ModuleConfiguration(moduleConfiguration.getWheelDiameter(),
//                                                                moduleConfiguration.getDriveReduction(),
//                                                                !moduleConfiguration.isDriveInverted(),
//                                                                moduleConfiguration.getSteerReduction(),
//                                                                !moduleConfiguration.isSteerInverted());
//
//        usedModuleConfiguration = actualModuleConfiguration;
//        usedModuleConfiguration = moduleConfiguration;

        // Derived from module details above
        double maxVelocityMetersPerSecond = kSparkMaxFreeSpeedRPM / 60.0 *
                                            moduleConfiguration.getDriveReduction() *
                                            moduleConfiguration.getWheelDiameter() * Math.PI;
        double maxAngularVelocityRadiansPerSecond = maxVelocityMetersPerSecond /
                                            Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

        super.setMaxVelocities(maxVelocityMetersPerSecond, maxAngularVelocityRadiansPerSecond);

        ShuffleboardLayout frontLeftModuleLayout = tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(0, 0);
        ShuffleboardLayout frontRightModuleLayout = tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(2, 0);
        ShuffleboardLayout backLeftModuleLayout = tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(4, 0);
        ShuffleboardLayout backRightModuleLayout = tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(6, 0);

        m_frontLeftModule = Mk4iSwerveModuleHelper.createNeo(
                frontLeftModuleLayout,
                theGearRatio,
                FRONT_LEFT_MODULE_DRIVE_MOTOR,
                FRONT_LEFT_MODULE_STEER_MOTOR,
                FRONT_LEFT_MODULE_STEER_ENCODER,
                FRONT_LEFT_MODULE_STEER_OFFSET
        );

        m_frontRightModule = Mk4iSwerveModuleHelper.createNeo(
                frontRightModuleLayout,
                theGearRatio,
                FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                FRONT_RIGHT_MODULE_STEER_MOTOR,
                FRONT_RIGHT_MODULE_STEER_ENCODER,
                FRONT_RIGHT_MODULE_STEER_OFFSET
        );

        m_backLeftModule = Mk4iSwerveModuleHelper.createNeo(
                backLeftModuleLayout,
                theGearRatio,
                BACK_LEFT_MODULE_DRIVE_MOTOR,
                BACK_LEFT_MODULE_STEER_MOTOR,
                BACK_LEFT_MODULE_STEER_ENCODER,
                BACK_LEFT_MODULE_STEER_OFFSET
        );

        m_backRightModule = Mk4iSwerveModuleHelper.createNeo(
                backRightModuleLayout,
                theGearRatio,
                BACK_RIGHT_MODULE_DRIVE_MOTOR,
                BACK_RIGHT_MODULE_STEER_MOTOR,
                BACK_RIGHT_MODULE_STEER_ENCODER,
                BACK_RIGHT_MODULE_STEER_OFFSET
        );

        frontLeftModuleLayout.addString("last error", () -> fl.getLastError().toString());
        frontRightModuleLayout.addString("last error", () -> fr.getLastError().toString());
        backLeftModuleLayout.addString("last error", () -> bl.getLastError().toString());
        backRightModuleLayout.addString("last error", () -> br.getLastError().toString());

        m_poseEstimator = new SwerveDrivePoseEstimator(
                m_kinematics,
                getGyroscopeRotation(),
                getPositions(),
                new Pose2d(new Translation2d(10, 10), getGyroscopeRotation())
        );
        tab.addNumber("yaw", () -> getGyroscopeRotation().getDegrees());
        SmartDashboard.putData(field);
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
        return m_poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        m_poseEstimator.resetPosition(getGyroscopeRotation(), getPositions(), pose);
    }

    public void periodic() {
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, m_maxVelocityMetersPerSecond);
        m_poseEstimator.update(getGyroscopeRotation(), getPositions());
        field.setRobotPose(getPose());

        m_frontLeftModule.set(-MAX_VOLTAGE * states[0].speedMetersPerSecond / m_maxVelocityMetersPerSecond, states[0].angle.getRadians());
        m_frontRightModule.set(-MAX_VOLTAGE * states[1].speedMetersPerSecond / m_maxVelocityMetersPerSecond, states[1].angle.getRadians());
        m_backLeftModule.set(-MAX_VOLTAGE * states[2].speedMetersPerSecond / m_maxVelocityMetersPerSecond, states[2].angle.getRadians());
        m_backRightModule.set(-MAX_VOLTAGE * states[3].speedMetersPerSecond / m_maxVelocityMetersPerSecond, states[3].angle.getRadians());
    }

    private void initEncoders() {
        fl = new CANCoder(FRONT_LEFT_MODULE_STEER_ENCODER);
        fr = new CANCoder(FRONT_RIGHT_MODULE_STEER_ENCODER);
        bl = new CANCoder(BACK_LEFT_MODULE_STEER_ENCODER);
        br = new CANCoder(BACK_RIGHT_MODULE_STEER_ENCODER);
        CANCoder[] arr = new CANCoder[]{fl, fr, bl, br};

        System.out.println("**********");
        System.out.println("** FL " + (Math.toDegrees(-FRONT_LEFT_MODULE_STEER_OFFSET)));
        System.out.println("** FR " + (Math.toDegrees(-FRONT_RIGHT_MODULE_STEER_OFFSET)));
        System.out.println("** BL " + (Math.toDegrees(-BACK_LEFT_MODULE_STEER_OFFSET)));
        System.out.println("** BR " + (Math.toDegrees(-BACK_RIGHT_MODULE_STEER_OFFSET)));
        System.out.println("**********");

        for (CANCoder tmp : arr) {
            System.out.println("**********");
            System.out.println("** ID  " + tmp.getDeviceID());
            System.out.println("** Abs " + tmp.getAbsolutePosition());
            System.out.println("** Pos " + tmp.getPosition());
            System.out.println("** Off " + tmp.configGetMagnetOffset());
            System.out.println("**********");
            tmp.configFactoryDefault();
        }

        System.out.println("\n**********");
        System.out.println("** Setting new offset ");
        System.out.println("**********\n");

        System.out.println(fl.configMagnetOffset(Math.toDegrees(FRONT_LEFT_MODULE_STEER_OFFSET), 50));
        System.out.println(fr.configMagnetOffset(Math.toDegrees(FRONT_RIGHT_MODULE_STEER_OFFSET), 50));
        System.out.println(bl.configMagnetOffset(Math.toDegrees(BACK_LEFT_MODULE_STEER_OFFSET), 50));
        System.out.println(br.configMagnetOffset(Math.toDegrees(BACK_RIGHT_MODULE_STEER_OFFSET), 50));

        for (CANCoder tmp : arr) {
            System.out.println("**********");
            System.out.println("** ID  " + tmp.getDeviceID());
            System.out.println("** Abs " + tmp.getAbsolutePosition());
            System.out.println("** Pos " + tmp.getPosition());
            System.out.println("** Off " + tmp.configGetMagnetOffset());
            System.out.println("**********");
        }
    }

}
