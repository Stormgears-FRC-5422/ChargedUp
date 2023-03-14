package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.CANCoder;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.CANSparkMax;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.ModuleConfiguration;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.*;
import frc.robot.RobotState;

import static frc.robot.constants.Constants.*;

public class SDSDrivetrain extends DrivetrainBase {
    /**
     * The maximum voltage that will be delivered to the drive motors.
     * <p>
     * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
     */
    public static final double MAX_VOLTAGE = 12.0;
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

    private CANCoder fl, fr, bl, br;

    // These are our modules. We initialize them in the constructor.
    // TODO use StormSparks with voltage and current safeties
    private final SwerveModule
            m_frontLeftModule, m_frontRightModule,
            m_backLeftModule, m_backRightModule;


    //pid gains from trapezoid move forward command
    //constraints made arbitrary

    private final PIDController xController = new PIDController(driveXkp, driveXki, 0.);
    private final PIDController yController = new PIDController(driveYkp, driveYki, 0.);
    private final PIDController rotController = new PIDController(turnkp, 0., 0.);
    final PPHolonomicDriveController m_holonomicController = new PPHolonomicDriveController(
            xController,
            yController,
            rotController
    );

    public SDSDrivetrain() {
        initEncoders();

        m_holonomicController.setTolerance(new Pose2d(0.05,  0.05, Rotation2d.fromDegrees(2.5)));

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

        // Derived from module details above
        double maxVelocityMetersPerSecond = kNeoFreeSpeedRPM / 60.0 *
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

        ((CANSparkMax) m_frontLeftModule.getDriveMotor()).setInverted(false);
        ((CANSparkMax) m_frontRightModule.getDriveMotor()).setInverted(false);
        ((CANSparkMax) m_backLeftModule.getDriveMotor()).setInverted(false);
        ((CANSparkMax) m_backRightModule.getDriveMotor()).setInverted(false);

        frontLeftModuleLayout.addString("last error", () -> fl.getLastError().toString());
        frontRightModuleLayout.addString("last error", () -> fr.getLastError().toString());
        backLeftModuleLayout.addString("last error", () -> bl.getLastError().toString());
        backRightModuleLayout.addString("last error", () -> br.getLastError().toString());

        frontLeftModuleLayout.addNumber("driveDistance()", m_frontLeftModule::getDriveDistance);
        frontRightModuleLayout.addNumber("driveDistance()", m_frontRightModule::getDriveDistance);
        backLeftModuleLayout.addNumber("driveDistance()", m_backLeftModule::getDriveDistance);
        backRightModuleLayout.addNumber("driveDistance()", m_backRightModule::getDriveDistance);

//        var pidTesting = Shuffleboard.getTab("Drivetrain PIDS");
//
//        pidTesting.add("X PID Controller", xController)
//                .withWidget(BuiltInWidgets.kPIDController)
//                .withPosition(4, 1);
//        pidTesting.add("Y PID Controller", yController)
//                .withWidget(BuiltInWidgets.kPIDController)
//                .withPosition(6, 1);
//        pidTesting.add("Rotation PID Controller", rotController)
//                .withWidget(BuiltInWidgets.kPIDController);
    }

    @Override
    public SwerveDriveKinematics getSwerveDriveKinematics() {
        return m_kinematics;
    }

    @Override
    public SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {
                m_frontLeftModule.getPosition(),
                m_frontRightModule.getPosition(),
                m_backLeftModule.getPosition(),
                m_backRightModule.getPosition()
        };
    }

    @Override
    public void goToPPTrajectoryState(PathPlannerTrajectory.PathPlannerState goalState) {
        var speeds = m_holonomicController.calculate(RobotState.getInstance().getCurrentPose(), goalState);
//        System.out.println("Current Chassis Speeds: " + speeds);
        drive(speeds, false);
    }

    @Override
    public boolean atReferenceState() {
        return m_holonomicController.atReference();
    }

    private void resetDriveEncoders() {
        ((CANSparkMax) m_frontLeftModule.getDriveMotor()).getEncoder().setPosition(0);
        ((CANSparkMax) m_frontRightModule.getDriveMotor()).getEncoder().setPosition(0);
        ((CANSparkMax) m_backLeftModule.getDriveMotor()).getEncoder().setPosition(0);
        ((CANSparkMax) m_backRightModule.getDriveMotor()).getEncoder().setPosition(0);
    }

    public void stormPeriodic() {
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, m_maxVelocityMetersPerSecond);

        m_frontLeftModule.set(MAX_VOLTAGE * states[0].speedMetersPerSecond / m_maxVelocityMetersPerSecond, states[0].angle.getRadians());
        m_frontRightModule.set(MAX_VOLTAGE * states[1].speedMetersPerSecond / m_maxVelocityMetersPerSecond, states[1].angle.getRadians());
        m_backLeftModule.set(MAX_VOLTAGE * states[2].speedMetersPerSecond / m_maxVelocityMetersPerSecond, states[2].angle.getRadians());
        m_backRightModule.set(MAX_VOLTAGE * states[3].speedMetersPerSecond / m_maxVelocityMetersPerSecond, states[3].angle.getRadians());
    }

    @Override
    public void enabledPeriodic() {
        RobotState.OdometryData currentOdometryData = new RobotState.OdometryData(
                getSwerveModulePositions(),
                getGyroscopeRotation()
        );
        RobotState.getInstance().setOdometryData(Timer.getFPGATimestamp(), currentOdometryData);
    }

    public void enabledInit() {
        resetDriveEncoders();
    }

    public void disabledInit() {
        resetDriveEncoders();
    }

    private void initEncoders() {
        fl = new CANCoder(FRONT_LEFT_MODULE_STEER_ENCODER);
        fr = new CANCoder(FRONT_RIGHT_MODULE_STEER_ENCODER);
        bl = new CANCoder(BACK_LEFT_MODULE_STEER_ENCODER);
        br = new CANCoder(BACK_RIGHT_MODULE_STEER_ENCODER);
        CANCoder[] arr = new CANCoder[]{fl, fr, bl, br};

//        System.out.println("**********");
//        System.out.println("** FL " + (Math.toDegrees(-FRONT_LEFT_MODULE_STEER_OFFSET)));
//        System.out.println("** FR " + (Math.toDegrees(-FRONT_RIGHT_MODULE_STEER_OFFSET)));
//        System.out.println("** BL " + (Math.toDegrees(-BACK_LEFT_MODULE_STEER_OFFSET)));
//        System.out.println("** BR " + (Math.toDegrees(-BACK_RIGHT_MODULE_STEER_OFFSET)));
//        System.out.println("**********");

        for (CANCoder tmp : arr) {
//            System.out.println("**********");
//            System.out.println("** ID  " + tmp.getDeviceID());
//            System.out.println("** Abs " + tmp.getAbsolutePosition());
//            System.out.println("** Pos " + tmp.getPosition());
//            System.out.println("** Off " + tmp.configGetMagnetOffset());
//            System.out.println("**********");
            tmp.configFactoryDefault();
        }

//        System.out.println("\n**********");
//        System.out.println("** Setting new offset ");
//        System.out.println("**********\n");
//
        System.out.println(fl.configMagnetOffset(Math.toDegrees(FRONT_LEFT_MODULE_STEER_OFFSET), 50));
        System.out.println(fr.configMagnetOffset(Math.toDegrees(FRONT_RIGHT_MODULE_STEER_OFFSET), 50));
        System.out.println(bl.configMagnetOffset(Math.toDegrees(BACK_LEFT_MODULE_STEER_OFFSET), 50));
        System.out.println(br.configMagnetOffset(Math.toDegrees(BACK_RIGHT_MODULE_STEER_OFFSET), 50));

//        for (CANCoder tmp : arr) {
//            System.out.println("**********");
//            System.out.println("** ID  " + tmp.getDeviceID());
//            System.out.println("** Abs " + tmp.getAbsolutePosition());
//            System.out.println("** Pos " + tmp.getPosition());
//            System.out.println("** Off " + tmp.configGetMagnetOffset());
//            System.out.println("**********");
//        }
    }
}
