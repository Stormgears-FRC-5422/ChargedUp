package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.SortedMap;

public class NavX extends SubsystemBase {
    private AHRS m_gyro;

    public NavX() {
        m_gyro = new AHRS(SPI.Port.kMXP);
    }

    public double getYaw() {
        return m_gyro.getYaw();
    }

    public double getPitch() {
        return m_gyro.getPitch();
    }

    public double getRoll() {
        return m_gyro.getRoll();
    }

    public boolean isMagnetometerCalibrated() {
        return m_gyro.isMagnetometerCalibrated();
    }

    public void zeroYaw() {
        m_gyro.zeroYaw();
    }

    public double getFusedHeading() {
        return m_gyro.getFusedHeading();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("yaw", getYaw());
        SmartDashboard.putNumber("roll", getRoll());
        SmartDashboard.putNumber("pitch", getPitch());
    }
}
