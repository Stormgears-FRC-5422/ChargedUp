package frc.robot.subsystems.arm;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ArmDriveKinematics {
    private ArmGeometry geometry;

    public ArmDriveKinematics(ArmGeometry geometry) {
        this.geometry = geometry;
    }

    public ArmJointSpeeds toJointSpeeds(ChassisSpeeds gripperSpeed) {
        return new ArmJointSpeeds(alphaSpeed(gripperSpeed.vxMetersPerSecond, gripperSpeed.vyMetersPerSecond),
                                  betaSpeed(gripperSpeed.vxMetersPerSecond, gripperSpeed.vyMetersPerSecond));
    }

    public double betaSpeed(double dx, double dy) {
        double l = geometry.l;
        double m = geometry.m;
        double se = geometry.alpha;
        double ee = geometry.beta;
        double betaSpeed = (dx * (l * Math.cos(se) + m * Math.sin(se - ee)) - dy * (-l * Math.sin(ee) + m * Math.sin(se + ee))) / (m * l * Math.sin(ee + se) * Math.cos(se) + m * l * Math.sin(ee - se) * Math.sin(se));
        return betaSpeed;
    }

    public double alphaSpeed(double dx, double dy) {
        double l = geometry.l;
        double m = geometry.m;
        double se = geometry.alpha;
        double ee = geometry.beta;
        double alphaSpeed = dx - m * Math.sin(ee + se) * ((dx * (l * Math.cos(se) + m * Math.sin(se - ee)) - dy * (-l * Math.sin(se) + m * Math.sin(se + ee))) / (m * l * Math.sin(ee + se) * Math.cos(se) + m * l * Math.sin(ee - se) * Math.sin(se)));
        return alphaSpeed;
    }

    public static void desaturateJointSpeeds(ArmJointSpeeds jointSpeeds, double max_dAlpha, double max_dBeta) {
        double div = Math.max(1.0, Math.max(jointSpeeds.dAlpha / max_dAlpha, jointSpeeds.dBeta / max_dBeta));

        jointSpeeds.dAlpha = jointSpeeds.dAlpha / div;
        jointSpeeds.dBeta = jointSpeeds.dBeta / div;
    }

}
