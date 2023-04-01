package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import frc.robot.constants.ShuffleboardConstants;
import frc.utils.motorcontrol.StormSpark;
import frc.utils.motorcontrol.StormTalon;
import java.lang.Math;

import com.revrobotics.SparkMaxPIDController;
import frc.utils.subsystemUtils.StormSubsystemBase;

import static frc.robot.constants.Constants.*;
import static frc.robot.constants.Constants.ArmConstants.*;

public class Arm extends StormSubsystemBase {
    final ArmGeometry m_geometry = new ArmGeometry();
    final StormSpark m_shoulder;
    SparkMaxPIDController m_shoulderPID;
    final StormTalon m_shoulderEncoder;
    final StormSpark m_elbow;
    SparkMaxPIDController m_elbowPID;
    final StormTalon m_elbowEncoder;
    double m_maxDAlpha;
    double m_maxDBeta;
    double m_maxXSpeed = 1.0;
    double m_maxYSpeed = 1.0;
    Pose2d gripperPose = new Pose2d();
    protected double forwardSoftLimit;
    protected double reverseSoftLimit;

    //TODO - delete these
    double dX = 0;
    double dY = 0;

    // The arm speeds will always be scaled by this factor. It defaults to kArmSpeedScale, but can be reset
    // (say by using the slider on the joystick)
    protected double m_armSpeedScale = 0;
    private int count = 0;

    private boolean m_useControllerPID = true;

    protected ChassisSpeeds m_gripperSpeed = new ChassisSpeeds(0.0, 0.0, 0.0);
    protected ArmJointSpeeds m_jointSpeeds = new ArmJointSpeeds(0.0, 0.0);
    private final ArmDriveKinematics m_kinematics;

    double forwardArmLimit;
    double backwardArmLimit;
    double upwardArmLimit;
    double downwardArmLimit;

    public Arm() {
        m_maxDAlpha = 2.0 * Math.PI * kNeoFreeSpeedRPM / (60.0 * armShoulderGearRatio);
        m_maxDBeta = 2.0 * Math.PI * kNeoFreeSpeedRPM / (60.0 * armElbowGearRatio);
        m_maxXSpeed = 1.0;
        m_maxYSpeed = 1.0;

        setSpeedScale(kArmSpeedScale);

        m_shoulderEncoder = new StormTalon(armShoulderEncoderID);
        // The shoulder encoder goes the wrong way and setSensorPhase doesn't seem to do anything. Not fighting this now
        setEncoderOffsetTicks(m_shoulderEncoder, -armShoulderEncoderOffsetTicks);
        m_shoulderEncoder.setNegatePosition(true);

        m_elbowEncoder = new StormTalon(armElbowEncoderID);
        setEncoderOffsetTicks(m_elbowEncoder, -armElbowEncoderOffsetTicks);

        m_shoulder = new StormSpark(armShoulderID, CANSparkMaxLowLevel.MotorType.kBrushless, StormSpark.MotorKind.kNeo);
        m_shoulder.setInverted(false);
        m_shoulder.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_shoulder.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(true);
        m_shoulder.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(true);
        m_shoulder.getEncoder().setPositionConversionFactor(2.0 * Math.PI / armShoulderGearRatio);
        m_shoulder.getEncoder().setPosition(m_shoulderEncoder.getPositionRadians(StormTalon.AngleRangeType.rangeNegToPos));

        m_elbow = new StormSpark(armElbowID, CANSparkMaxLowLevel.MotorType.kBrushless, StormSpark.MotorKind.kNeo);
        m_elbow.setInverted(true);
        m_elbow.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_elbow.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(true);
        m_elbow.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(true);
        m_elbow.getEncoder().setPositionConversionFactor(2.0 * Math.PI / armElbowGearRatio);
        m_elbow.getEncoder().setPosition(m_elbowEncoder.getPositionRadians(StormTalon.AngleRangeType.rangeNegToPos));

        forwardArmLimit = forwardConstraint + armInlay
                          - gripperWheelRadius - armForwardSafetyBuffer;
        backwardArmLimit = backwardConstraint
                           + gripperWheelRadius + armBackwardSafetyBuffer;

        upwardArmLimit = upwardConstraint - chassisHeight
                         - gripperWheelRadius - armUpwardSafetyBuffer;
        downwardArmLimit = downwardConstraint
                           + gripperWheelRadius + armDownwardSafetyBuffer;

        try {
            System.out.println("Delete this quick nap - only for simulation");
            Thread.sleep(1000);
        } catch (Exception e) {
            e.printStackTrace();
        }

        if (m_useControllerPID) {
            // PID coefficients
            double kP = 0.000;
            double kI = 0;
            double kD = 0;
            double kIz = 0;
            double kFF = 1;
            double kMaxOutput = 1;
            double kMinOutput = -1;

            m_shoulderPID = m_shoulder.getPIDController();
            m_shoulderPID.setI(kI);
            m_shoulderPID.setD(kD);
            m_shoulderPID.setP(kP);
            m_shoulderPID.setIZone(kIz);
            m_shoulderPID.setFF(kFF);
            m_shoulderPID.setOutputRange(kMinOutput, kMaxOutput);

            m_elbowPID = m_elbow.getPIDController();
            m_elbowPID.setP(kP);
            m_elbowPID.setI(kI);
            m_elbowPID.setD(kD);
            m_elbowPID.setIZone(kIz);
            m_elbowPID.setFF(kFF);
            m_elbowPID.setOutputRange(kMinOutput, kMaxOutput);
        }

        updateGeometry();
        m_kinematics = new ArmDriveKinematics(m_geometry);

        //setShoulderSoftLimits(2 * Math.PI/3, Math.PI/6);
        setShoulderSoftLimits((113./180.)*Math.PI, (40./180.)*Math.PI);
        setElbowSoftLimits((-5./180.)*Math.PI, (-165./180.)*Math.PI);
        enableSoftLimits();

        if (Toggles.useLogs) {
            ShuffleboardConstants.getInstance().armStatusLayout
                    .addNumber("x", () -> gripperPose.getX());
            ShuffleboardConstants.getInstance().armStatusLayout
                    .addNumber("y", () -> gripperPose.getY());
            ShuffleboardConstants.getInstance().armStatusLayout
                    .addNumber("vx", () -> dX);
            ShuffleboardConstants.getInstance().armStatusLayout
                    .addNumber("vy", () -> dY);
            ShuffleboardConstants.getInstance().armStatusLayout
                    .addNumber("alpha", m_geometry::getAlpha);
            ShuffleboardConstants.getInstance().armStatusLayout
                    .addNumber("beta", m_geometry::getBeta);
            ShuffleboardConstants.getInstance().armStatusLayout
                    .addNumber("valpha", () -> m_jointSpeeds.dAlpha);
            ShuffleboardConstants.getInstance().armStatusLayout
                    .addNumber("vbeta", () -> m_jointSpeeds.dBeta);
        }
    }
    
    public void disableSoftLimits() {
        m_shoulder.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
        m_shoulder.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
        m_elbow.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
        m_elbow.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
        //System.out.println(getName() + ".disableSoft()");
    }

    public void enableSoftLimits() {
        m_shoulder.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        m_shoulder.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        m_elbow.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        m_elbow.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        //System.out.println(getName() + ".enableLimits()");
    }

    public void setShoulderSoftLimits(double forward, double reverse) {
        forwardSoftLimit = forward;
        reverseSoftLimit = reverse;
        m_shoulder.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) forwardSoftLimit);
        m_shoulder.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) reverseSoftLimit);
    }

    public void setElbowSoftLimits(double forward, double reverse) {
        forwardSoftLimit = forward;
        reverseSoftLimit = reverse;
        m_elbow.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) forwardSoftLimit);
        m_elbow.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) reverseSoftLimit);
    }

    private void updateGeometry() {
        double s = m_shoulder.getEncoder().getPosition();
        double e = m_elbow.getEncoder().getPosition();
//        System.out.println("S: " + s + " ; E: " + e + ", ecf = " + m_elbow.getEncoder().getPositionConversionFactor());
        m_geometry.setAngles(s, e);
    }

    private void updateTargetGeometry(ArmJointSpeeds jointSpeeds) {
        double s = m_geometry.getAlpha() + jointSpeeds.dAlpha * 0.02;
        double e = m_geometry.getBeta() + jointSpeeds.dBeta * 0.02;
        e = Math.min(e,-5 * (Math.PI/180));
        e = Math.max(e, -Math.PI);
        s = Math.max(s, Math.PI/6);
        s = Math.min(s, 2 * Math.PI/3);
//        System.out.println("S: " + s + " E: " + e);
        m_geometry.setAngles(s,e);
    }

    @Override
    public void stormPeriodic() {
        // TODO - should be a config, but it is really always 12
        double MAX_VOLTAGE = 12.0;

        double s = m_jointSpeeds.dAlpha / m_maxDAlpha;
        double e = m_jointSpeeds.dBeta / m_maxDBeta;

        if (m_useControllerPID) {
            // TODO - consider adding the FeedForward for target voltage
            m_shoulderPID.setReference(s, CANSparkMax.ControlType.kVelocity);
            m_elbowPID.setReference(e, CANSparkMax.ControlType.kVelocity);
        } else {
            m_shoulder.setVoltage(s * MAX_VOLTAGE);
            m_elbow.setVoltage(e * MAX_VOLTAGE);
        }

//        System.out.println("Shoulder V: " + s * MAX_VOLTAGE);
//        System.out.println("Elbow V: " + e * MAX_VOLTAGE);

       updateGeometry();

        // fake geometry
//         updateTargetGeometry(m_jointSpeeds);

        gripperPose = m_geometry.getPose();
//        SmartDashboard.putNumber("X-coordinate", gripperPose.getX());
//        SmartDashboard.putNumber("Y-coordinate: ", gripperPose.getY());
//
//        SmartDashboard.putNumber("X-coordinate in arm space", gripperPose.getX());
//        SmartDashboard.putNumber("Y-coordinate in arm space", gripperPose.getY());
//        SmartDashboard.putNumber("dX arm space", dX);
//        SmartDashboard.putNumber("dY arm space", dY);
//        SmartDashboard.putNumber("Alpha angle in arm space", m_geometry.getAlpha());
//        SmartDashboard.putNumber("Beta in arm space", m_geometry.getBeta());
//        SmartDashboard.putNumber("dAlpha arm space", s);
//        SmartDashboard.putNumber("dBeta arm space", e);
    }


    public void moveArm(ArmJointSpeeds jointSpeeds) {
        // Scale incoming speeds
        m_jointSpeeds = jointSpeeds.scale(m_armSpeedScale);
    }

    public void percentOutMoveArm(ArmJointSpeeds jointSpeeds) {
        moveArm(new ArmJointSpeeds(jointSpeeds.dAlpha * m_maxDAlpha,
                jointSpeeds.dBeta * m_maxDBeta));
    }

    public void xyMoveArm(ChassisSpeeds speeds) {
        regulateArm(gripperPose, speeds);
        dX = speeds.vxMetersPerSecond;
        dY = speeds.vyMetersPerSecond;
        ArmJointSpeeds jointSpeeds  = m_kinematics.toJointSpeeds(speeds);
        m_jointSpeeds = ArmDriveKinematics.desaturateJointSpeeds(jointSpeeds, m_maxDAlpha, m_maxDBeta);
        m_jointSpeeds.scale(m_armSpeedScale);
//        System.out.println("alpha: " + m_jointSpeeds.dAlpha + " beta: " + m_jointSpeeds.dBeta);
    }

    public void percentOutXYMoveArm(ChassisSpeeds speeds) {
        xyMoveArm(new ChassisSpeeds(speeds.vxMetersPerSecond * m_maxXSpeed,
                                    speeds.vyMetersPerSecond* m_maxYSpeed,
                                0.0));
    }

    public void stopArm() {
        moveArm(new ArmJointSpeeds(0, 0));
    }

    private void setEncoderOffsetTicks(StormTalon talon, int offset) {
        talon.setOffsetRadians(2. * Math.PI * offset / kMagEncoderTicksPerRotation);
    }

    public void setSpeedScale(double scale) {
        m_armSpeedScale = MathUtil.clamp(scale, 0, kArmSpeedScale);
    }

    public void regulateArm(Pose2d gripperCoords, ChassisSpeeds speeds){
        // forward / backward
        if (gripperCoords.getX() >= forwardArmLimit && speeds.vxMetersPerSecond > 0) {
//            System.out.println("limiting forward x at x=" + gripperCoords.getX() + ", dx=" + speeds.vxMetersPerSecond);
            speeds.vxMetersPerSecond = 0;
            // speeds.vxMetersPerSecond = Math.min(m_maxXSpeed * m_armSpeedScale * applyCushionX(speeds, gripperCoords), speeds.vxMetersPerSecond);
        } else if (gripperCoords.getX() <= backwardArmLimit && speeds.vxMetersPerSecond < 0) {
//            System.out.println("limiting backward x at x=" + gripperCoords.getX() + ", dx=" + speeds.vxMetersPerSecond);
            speeds.vxMetersPerSecond = 0;
            // speeds.vxMetersPerSecond = Math.max(m_maxXSpeed * m_armSpeedScale * applyCushionX(speeds, gripperCoords), speeds.vxMetersPerSecond);
        }

        // upward / downward
        if (gripperCoords.getY() >= upwardArmLimit && speeds.vyMetersPerSecond > 0) {
//            System.out.println("limiting upward y at y=" + gripperCoords.getY() + ", dy=" + speeds.vyMetersPerSecond);
            speeds.vyMetersPerSecond = 0;
            // speeds.vyMetersPerSecond = Math.min(m_maxYSpeed * m_armSpeedScale * applyCushionY(speeds, gripperCoords),speeds.vyMetersPerSecond);
        } else if (gripperCoords.getY() <= downwardArmLimit && speeds.vyMetersPerSecond < 0) {
//            System.out.println("limiting downward y at y=" + gripperCoords.getY() + ", dy=" + speeds.vyMetersPerSecond);
            speeds.vyMetersPerSecond = 0;
            // speeds.vyMetersPerSecond = Math.max(m_maxYSpeed * m_armSpeedScale * applyCushionY(speeds, gripperCoords), speeds.vyMetersPerSecond);
        }
    }

//    public double applyCushionX(ChassisSpeeds speed, Pose2d position) {
//        double delta;
//
//        if (Math.abs(position.getX() - forwardArmLimit) < 0.1016) { //0.1016 = 4 inches, change later
//            delta = position.getX() - forwardArmLimit;
//        }
//        else if (Math.abs(position.getX()- backwardArmLimit) < 0.1016) {
//            delta = Math.abs(position.getX() - backwardArmLimit);
//        }
//        else{
//            return speed.vxMetersPerSecond;
//        }
//
//        double limit = delta/0.1016;
//
//        if (Math.abs(speed.vxMetersPerSecond) < limit) {
//            return speed.vxMetersPerSecond;
//        }
//
//        return limit;
//
//    }
//
//    public double applyCushionY(ChassisSpeeds speed, Pose2d position) {
//        double delta;
//
//        if (Math.abs(position.getY() - upwardArmLimit) < 0.1016) { //0.1016 = 4 inches, change later
//            delta = position.getY() - upwardArmLimit;
//        }
//        else if (Math.abs(position.getY()- downwardArmLimit) < 0.1016) {
//            delta = Math.abs(position.getY() - downwardArmLimit);
//        }
//        else{
//            return speed.vyMetersPerSecond;
//        }
//
//        double limit = delta/0.1016;
//
//        if (Math.abs(speed.vyMetersPerSecond) < limit) {
//            return speed.vyMetersPerSecond;
//        }
//
//        return limit;
//
//    }


    public Pose2d getGripperPose() {
        return gripperPose;
    }

    public Translation3d getGlobalTranslation() {
        return new Translation3d(
                gripperPose.getX() + armTranslation.getX(), 0,
                gripperPose.getY() + armTranslation.getY()
        );
    }

    /** returns a translation in arm space given a translation from robot origin */
    public static Translation2d fromGlobalTranslation(Translation3d global) {
        return new Translation2d(
                fromGlobalX(global.getX()),
                fromGlobalHeight(global.getZ())
        );
    }

    public static double fromGlobalX(double global) {
        return global - armTranslation.getX();
    }

    public static double fromGlobalHeight(double global) {
        return global - armTranslation.getY();
    }
}
