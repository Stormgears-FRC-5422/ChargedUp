package frc.robot.subsystems.arm;

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

import static frc.robot.Constants.*;

public class Arm extends SubsystemBase {
    ArmGeometry m_geometry = new ArmGeometry();
    StormSpark m_shoulder;
    StormTalon m_shoulderEncoder;
    StormSpark m_elbow;
    StormTalon m_elbowEncoder;

    double m_maxDAlpha;
    double m_maxDBeta;
    double m_maxXSpeed = 1.0;
    double m_maxYSpeed = 1.0;

    // The arm speeds will always be scaled by this factor. It defaults to kArmSpeedScale, but can be reset
    // (say by using the slider on the joystick)
    protected double m_armSpeedScale = 0;
    private int count = 0;

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
        m_shoulder.getEncoder().setPosition(m_shoulderEncoder.getPositionRadians());

        m_elbow = new StormSpark(armElbowID, CANSparkMaxLowLevel.MotorType.kBrushless, StormSpark.MotorKind.kNeo);
        m_elbow.setInverted(true);
        m_elbow.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(true);
        m_elbow.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(true);
        m_elbow.getEncoder().setPositionConversionFactor(2.0 * Math.PI / armElbowGearRatio);
        m_elbow.getEncoder().setPosition(m_elbowEncoder.getPositionRadians());

        updateGeometry();
        m_kinematics = new ArmDriveKinematics(m_geometry);
    }

    private void updateGeometry() {
        m_geometry.setAngles(m_shoulder.getEncoder().getPosition(), m_elbow.getEncoder().getPosition());
    }

    @Override
    public void periodic() {
        double MAX_VOLTAGE = 12.0;

        double s = MAX_VOLTAGE * m_jointSpeeds.dAlpha / m_maxDAlpha;
        double e = MAX_VOLTAGE * m_jointSpeeds.dBeta / m_maxDBeta;

        m_shoulder.setVoltage(s);
        m_elbow.setVoltage(e);

        updateGeometry();

        Pose2d pose = m_geometry.getPose();
        SmartDashboard.putNumber("X-coordinate in arm space", pose.getX());
        SmartDashboard.putNumber("Y-coordinate in arm space", pose.getY());
        SmartDashboard.putNumber("Alpha angle in arm space", m_geometry.getAlpha());
        SmartDashboard.putNumber("Beta in arm space", m_geometry.getBeta());
    }


    public void moveArm(ArmJointSpeeds speeds) {
        // Scale incoming speeds
        ArmJointSpeeds s = new ArmJointSpeeds(
                m_armSpeedScale * speeds.dAlpha,
                m_armSpeedScale * speeds.dBeta);

        m_jointSpeeds = s;
    }

    public void percentOutMoveArm(ArmJointSpeeds speeds) {
        moveArm(new ArmJointSpeeds(speeds.dAlpha * m_maxDAlpha,
                speeds.dBeta * m_maxDBeta));
    }

    public void xyMoveArm(ChassisSpeeds speeds) {
        ArmJointSpeeds jointSpeeds  = m_kinematics.toJointSpeeds(speeds);
        ArmDriveKinematics.desaturateJointSpeeds(jointSpeeds, m_maxDAlpha, m_maxDBeta);

        m_jointSpeeds = jointSpeeds;
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
