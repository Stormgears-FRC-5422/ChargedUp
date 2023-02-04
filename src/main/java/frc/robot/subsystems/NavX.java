package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class NavX extends SubsystemBase {

    private ShuffleboardTab navXtab = Shuffleboard.getTab("NavX");



    private AHRS m_gyro;
    public NavX() {
        switch (navXConnection) {
            case "SPI":
                m_gyro = new AHRS(SPI.Port.kMXP);
                break;
            case "USB":
                m_gyro = new AHRS(SerialPort.Port.kUSB);
                break;
            default:
                m_gyro = new AHRS(SPI.Port.kMXP);
                 System.out.println("NO NavX Connection Given. Defauly NavX connection used: SPI");
                break;
        }
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
}
