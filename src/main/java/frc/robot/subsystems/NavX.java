package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NavX extends SubsystemBase {

    private AHRS m_gyro;


    public NavX() {
        m_gyro = new AHRS(SPI.Port.kMXP);
    }

    public double getYaw() {
        return m_gyro.getYaw();
    }

    public void zeroYaw() {
        m_gyro.zeroYaw();
    }


    public double getPitch() {
        return m_gyro.getPitch();
    }

    public double getRoll() {
        return m_gyro.getRoll();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("yaw", getYaw());
        SmartDashboard.putNumber("pitch", getPitch());
        SmartDashboard.putNumber("roll", getRoll());
        String uphillDirection = "flat";

        if(getPitch() > 5) {
                uphillDirection = "Forward";
            }
        if (getPitch() < -5) {
            uphillDirection = "Backward";
                }
        if (getRoll() < -5) {
            uphillDirection = "Left";
        }
        if (getRoll() > 5) {
            uphillDirection = "Right";
        }

        SmartDashboard.putString("uphillDirection", uphillDirection);
    }
}