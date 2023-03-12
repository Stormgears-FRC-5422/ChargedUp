package frc.robot.subsystems.arm;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

import static java.lang.Math.*;

public class ArmDriveKinematics {
    private final ArmGeometry geometry;

    public ArmDriveKinematics(ArmGeometry geometry) {
        this.geometry = geometry;
    }

    public ArmJointSpeeds toJointSpeeds(ChassisSpeeds gripperSpeed) {
        return new ArmJointSpeeds(alphaSpeed(gripperSpeed.vxMetersPerSecond, gripperSpeed.vyMetersPerSecond),
                betaSpeed(gripperSpeed.vxMetersPerSecond, gripperSpeed.vyMetersPerSecond));
    }

    public double alphaSpeed(double dx, double dy) {
        double l = geometry.l;
        double m = geometry.m;
        double se = geometry.alpha;
        double ee = geometry.beta;
//        double alphaSpeed = dx - m * sin(ee + se) * ((dx * (l * cos(se) + m * sin(se - ee)) - dy * (-l * sin(se) + m * sin(se + ee))) /
//                                 (m * l * sin(ee + se) * cos(se) + m * l * sin(ee - se) * sin(se)));

        double alphaNum = dx * (m * cos(se + ee)) + dy * (m * sin(se + ee));
        double den = l * m * sin(ee);
        // TODO - handle singularity when den --> 0
        return alphaNum / den;
    }

    public double betaSpeed(double dx, double dy) {
        double l = geometry.l;
        double m = geometry.m;
        double se = geometry.alpha;
        double ee = geometry.beta;
//        double betaSpeed = (dx * (l * cos(se) + m * sin(se - ee)) - dy * (-l * sin(ee) + m * sin(se + ee))) /
//                                 (m * l * sin(ee + se) * cos(se) + m * l * sin(ee - se) * sin(se));

        double betaNum = dx * (-l * cos(se) - m * cos(se + ee)) + dy * (-l * sin(se) - m * sin(se + ee));
        double den = l * m * sin(ee);
        // TODO - handle singularity when den --> 0
        return betaNum / den;
    }


    public static ArmJointSpeeds desaturateJointSpeeds(ArmJointSpeeds jointSpeeds, double max_dAlpha, double max_dBeta) {
        double div = max(1.0, max(abs(jointSpeeds.dAlpha) / max_dAlpha, abs(jointSpeeds.dBeta) / max_dBeta));

        return jointSpeeds.scale(1.0 / div);
    }

}
