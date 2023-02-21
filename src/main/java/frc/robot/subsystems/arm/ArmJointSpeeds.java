package frc.robot.subsystems.arm;

public class ArmJointSpeeds {
    double shoulderOmegaRadiansPerSecond = 0;
    double elbowOmegaRadiansPerSecond = 0;

    public ArmJointSpeeds() {
        this(0.0,0.0);
    }

    public ArmJointSpeeds(double shoulderOmegaRadiansPerSecond, double elbowOmegaRadiansPerSecond) {
        this.shoulderOmegaRadiansPerSecond = shoulderOmegaRadiansPerSecond;
        this.elbowOmegaRadiansPerSecond = elbowOmegaRadiansPerSecond;
    }

}
