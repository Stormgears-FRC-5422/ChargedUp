package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.motorcontrol.StormSpark;
import frc.utils.motorcontrol.StormTalon;
import java.lang.Math;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.SparkMaxPIDController;

import static frc.robot.Constants.*;
import static java.lang.Thread.sleep;

public class Arm extends SubsystemBase {
    ArmGeometry m_geometry = new ArmGeometry();
    StormSpark m_shoulder;
    SparkMaxPIDController m_shoulderPID;
    StormTalon m_shoulderEncoder;
    StormSpark m_elbow;
    SparkMaxPIDController m_elbowPID;
    StormTalon m_elbowEncoder;
    double m_maxDAlpha;
    double m_maxDBeta;
    double m_maxXSpeed = 1.0;
    double m_maxYSpeed = 1.0;

    //TODO - delete these
//    double dX = 0;
//    double dY = 0;

    // The arm speeds will always be scaled by this factor. It defaults to kArmSpeedScale, but can be reset
    // (say by using the slider on the joystick)
    protected double m_armSpeedScale = 0;
    private int count = 0;

    private boolean m_useControllerPID = true;

    protected ChassisSpeeds m_gripperSpeed = new ChassisSpeeds(0.0, 0.0, 0.0);
    protected ArmJointSpeeds m_jointSpeeds = new ArmJointSpeeds(0.0, 0.0);
    private ArmDriveKinematics m_kinematics;

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
        m_shoulder.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(true);
        m_shoulder.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(true);
        m_shoulder.getEncoder().setPositionConversionFactor(2.0 * Math.PI / armShoulderGearRatio);
        m_shoulder.getEncoder().setPosition(m_shoulderEncoder.getPositionRadians(StormTalon.AngleRangeType.rangeNegToPos));

        m_elbow = new StormSpark(armElbowID, CANSparkMaxLowLevel.MotorType.kBrushless, StormSpark.MotorKind.kNeo);
        m_elbow.setInverted(true);
        m_elbow.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(true);
        m_elbow.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(true);
        m_elbow.getEncoder().setPositionConversionFactor(2.0 * Math.PI / armElbowGearRatio);
        m_elbow.getEncoder().setPosition(m_elbowEncoder.getPositionRadians(StormTalon.AngleRangeType.rangeNegToPos));

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
        m_geometry.setAngles(s,e);
    }

    @Override
    public void periodic() {
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

        updateGeometry();

        // fake geometry
//        updateTargetGeometry(m_jointSpeeds);

        Pose2d pose = m_geometry.getPose();

//        SmartDashboard.putNumber("X-coordinate in arm space", pose.getX());
//        SmartDashboard.putNumber("Y-coordinate in arm space", pose.getY());
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
//        dX = speeds.vxMetersPerSecond;
//        dY = speeds.vyMetersPerSecond;
        ArmJointSpeeds jointSpeeds  = m_kinematics.toJointSpeeds(speeds);
        m_jointSpeeds = ArmDriveKinematics.desaturateJointSpeeds(jointSpeeds, m_maxDAlpha, m_maxDBeta);
        m_jointSpeeds.scale(m_armSpeedScale);
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
}
