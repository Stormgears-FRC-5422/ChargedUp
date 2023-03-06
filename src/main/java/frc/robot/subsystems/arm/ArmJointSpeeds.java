package frc.robot.subsystems.arm;

public class ArmJointSpeeds {
    public double dAlpha = 0;
    public double dBeta = 0;

    public ArmJointSpeeds() {
        this(0.0,0.0);
    }

    public ArmJointSpeeds(double dAlpha, double dBeta) {
        this.dAlpha = dAlpha;
        this.dBeta = dBeta;
    }

}
