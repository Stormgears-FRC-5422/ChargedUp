package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.RobotState;
import frc.robot.constants.ShuffleboardConstants;
import frc.utils.subsystemUtils.StormSubsystemBase;

import static frc.robot.constants.Constants.navXConnection;

public class NavX extends StormSubsystemBase {

    private final AHRS m_gyro;

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
                 System.out.println("NO NavX Connection Given. Default NavX connection used: SPI");
                break;
        }

        ShuffleboardTab tab = ShuffleboardConstants.getInstance().navXTab;
        tab.addNumber("yaw", this::getYaw);
        tab.addNumber("Absolute Yaw", () -> getAbsoluteRotation().getDegrees());
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

    /** get absolute rotation (-180, 180) inverted so counter-clockwise is positive */
    public Rotation2d getAbsoluteRotation() {
        double invertedYaw = -1 * getYaw();
        // get absolute
        double absolute0To360 = (invertedYaw + 180.0) % 360;
        // subtract 180 again
        return Rotation2d.fromDegrees((absolute0To360 - 180.0));
    }

    @Override
    public void stormPeriodic() {
        RobotState.getInstance().addGyroData(Timer.getFPGATimestamp(), getAbsoluteRotation());
    }

    public void enabledInit() {
        zeroYaw();
    }
}
