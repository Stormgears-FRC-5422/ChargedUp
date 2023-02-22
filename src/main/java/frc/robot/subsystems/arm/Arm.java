package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.motorcontrol.StormSpark;
import frc.utils.motorcontrol.StormTalon;

import static frc.robot.Constants.*;

public class Arm extends SubsystemBase {
    public StormSpark shoulder;
    public StormSpark elbow;
    public StormTalon shoulderEncoder;
    public StormTalon elbowEncoder;

    public double m_maxShoulderOmegaRadiansPerSecond;
    public double m_maxElbowOmegaRadiansPerSecond;

    // The arm Joint speeds will always be scaled by this factor. It defaults to kArmSpeedScale, but can be reset
    // (say by using the slider on the joystick)
    protected double m_armSpeedScale = 0;

    protected ArmJointSpeeds m_jointSpeeds = new ArmJointSpeeds(0.0, 0.0);

    public Arm() {
        double maxShoulderOmegaRadiansPerSecond = 2.0 * Math.PI * kNeoFreeSpeedRPM / (60.0 * armShoulderGearRatio);
        double maxElbowOmegaRadiansPerSecond = 2.0 * Math.PI * kNeoFreeSpeedRPM / (60.0 * armElbowGearRatio);

        setSpeedScale(kArmSpeedScale);
        setMaxAngularVelocities(maxShoulderOmegaRadiansPerSecond, maxElbowOmegaRadiansPerSecond);

        shoulder = new StormSpark(armShoulderID, CANSparkMaxLowLevel.MotorType.kBrushless, StormSpark.MotorKind.kNeo);
        elbow = new StormSpark(armElbowID, CANSparkMaxLowLevel.MotorType.kBrushless, StormSpark.MotorKind.kNeo);

        shoulderEncoder = new StormTalon(armShoulderEncoderID);
        elbowEncoder = new StormTalon(armElbowEncoderID);
        setEncoderOffsetTicks(shoulderEncoder, armShoulderEncoderOffsetTicks);
        setEncoderOffsetTicks(elbowEncoder, armElbowEncoderOffsetTicks);
    }

    @Override
    public void periodic() {
        double MAX_VOLTAGE = 12.0;

        shoulder.setVoltage(MAX_VOLTAGE * m_jointSpeeds.shoulderOmegaRadiansPerSecond / m_maxShoulderOmegaRadiansPerSecond);
        elbow.setVoltage(MAX_VOLTAGE * m_jointSpeeds.elbowOmegaRadiansPerSecond / m_maxElbowOmegaRadiansPerSecond);
    }

    public void moveArm(ArmJointSpeeds speeds) {
        // Scale incoming speeds
        ArmJointSpeeds s = new ArmJointSpeeds(
                m_armSpeedScale * speeds.shoulderOmegaRadiansPerSecond,
                m_armSpeedScale * speeds.elbowOmegaRadiansPerSecond);

        m_jointSpeeds = s;
    }

    public void percentOutMoveArm(ArmJointSpeeds speeds) {
        moveArm(new ArmJointSpeeds(speeds.shoulderOmegaRadiansPerSecond * m_maxShoulderOmegaRadiansPerSecond,
                speeds.elbowOmegaRadiansPerSecond * m_maxElbowOmegaRadiansPerSecond));
    }

    public void stopArm() {
        moveArm(new ArmJointSpeeds(0, 0));
    }

    private void setEncoderOffsetTicks(StormTalon talon, int offset) {
        talon.setOffsetRadians(2. * Math.PI * offset / magEncoderTicksPerRotation);
    }

    // TODO - what are the best natural units for this? ticks, radians, degrees?
    private double getEncoderAbsolutePositionDegrees(StormTalon talon) {
        return 360. * talon.getPositionTicks() / magEncoderTicksPerRotation;
    }

    private void setMaxAngularVelocities(double maxShoulderOmegaRadiansPerSecond, double maxElbowOmegaRadiansPerSecond) {
        this.m_maxShoulderOmegaRadiansPerSecond = maxShoulderOmegaRadiansPerSecond;
        this.m_maxElbowOmegaRadiansPerSecond = maxElbowOmegaRadiansPerSecond;
    }


    public void setSpeedScale(double scale) {
        m_armSpeedScale = MathUtil.clamp(scale, 0, kArmSpeedScale);
    }


/*
    public ChassisSpeeds getCurrentChassisSpeeds() {
        return m_chassisSpeeds;
    }
    public abstract SwerveDriveKinematics getSwerveDriveKinematics();
    public abstract SwerveModulePosition[] getSwerveModulePositions();
    public abstract void goToTrajectoryState(Trajectory.State goalState);
    public abstract void goToPPTrajectoryState(PathPlannerTrajectory.PathPlannerState goalState);
    public abstract void onEnable();
    public abstract void onDisable();
*/
}
