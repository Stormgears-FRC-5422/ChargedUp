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
    private double offset = 0.0;

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
        offset = 0.0;
    }

    /** angle must be (-180, 180) such that counter-clockwise if positive */
    public void setAngle(double angle) {
        zeroYaw();
        offset = angle;
    }

    public double getFusedHeading() {
        return m_gyro.getFusedHeading();
    }

    /** get absolute rotation (-180, 180) inverted so counter-clockwise is positive with offset */
    public Rotation2d getAbsoluteRotation() {
        double invertedYaw = -1.0 * getYaw();
        double abs0to360 = invertedYaw + 180.0;
        double offsetModified = offset + 360.0;
        abs0to360 += offsetModified;
        abs0to360 = (abs0to360 > 360)? abs0to360 % 360 : abs0to360;
        return Rotation2d.fromDegrees(abs0to360 - 180.0);
    }

    @Override
    public void enabledPeriodic() {
        RobotState.getInstance().addGyroData(Timer.getFPGATimestamp(), getAbsoluteRotation());
    }

    public void enabledInit() {
        setAngle(RobotState.getInstance().getStartPose().getRotation().getDegrees());
    }
}
