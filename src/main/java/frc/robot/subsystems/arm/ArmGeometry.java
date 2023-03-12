package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import static frc.robot.constants.Constants.*;
import static java.lang.Math.*;
public class ArmGeometry {
    double alpha, beta, l, m;
    // Pose in arm-based coordinates. The shoulder joint is at (0,0)
    Pose2d m_endPose;
    Rotation2d noRotation = new Rotation2d(0.0);

    public ArmGeometry() {
        l = kA1Length;
        m = kA2Length;

        m_endPose = new Pose2d(0,0,noRotation);
    }

    public void setPose(Pose2d pose) {
        m_endPose = pose;
        _onUpdateAngles();
    }

    public void setAngles(double alpha, double beta) {
        this.alpha = alpha;
        this.beta = beta;
        _onUpdateAngles();
    }

    public Pose2d getPose() {
        return m_endPose;
    }

    public double getAlpha() {
        return alpha;
    }

    public double getBeta() {
        return beta;
    }

    // Called internally when the angles have been updated
    private void _onUpdateAngles() {
        m_endPose = new Pose2d( m * cos(alpha + beta) + l * cos(alpha),
                                m * sin(alpha + beta) + l * sin(alpha),
                                noRotation);
    }

    // Called internally when the pose has been updated
    private void _onUpdateXY() {
        double x = m_endPose.getX();
        double y = m_endPose.getY();

        alpha = atan2(y,x) -
                atan2( (m * sin(beta)),
                       (l + m * cos(beta)));

        beta = acos( ( x*x + y*y - l * l - m * m) /
                     (2.0 * l * m));
    }

}
