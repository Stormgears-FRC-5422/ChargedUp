package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

import static frc.robot.Constants.*;

public class NavX extends SubsystemBase {

    private ShuffleboardTab navXtab = Shuffleboard.getTab("NavX");

    private AHRS m_gyro;
    public NavX() {
        switch (navXConnection) {
            case "SPI":
                m_gyro = new AHRS(SPI.Port.kMXP);
                System.out.println("SPI");
                break;
            case "USB":
                m_gyro = new AHRS(SerialPort.Port.kUSB);
                break;
            default:
                m_gyro = new AHRS(SPI.Port.kMXP);
                 System.out.println("NO NavX Connection Given. Defauly NavX connection used: SPI");
                break;
        }

        navXtab.addNumber("yaw", this::getYaw);
        navXtab.addNumber("pitch", this::getPitch);
        navXtab.addNumber("roll", this::getRoll);
        navXtab.addBoolean("isMagnetometerCalibrated", this::isMagnetometerCalibrated);
        navXtab.addNumber("fusedHeading", this::getFusedHeading);
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
        RobotState.getInstance().setCurrentGyroRotation(Rotation2d.fromDegrees(360.0 - getYaw()));
    }
}
