package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

import static frc.robot.Constants.navXConnection;

public class NavX extends SubsystemBase implements IEnabledDisabled {

    private final AHRS m_gyro;
    //Offset degrees set at start of match
    private double yawOffset = 0.0;

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

        ShuffleboardTab navXtab = Shuffleboard.getTab("NavX");
        navXtab.addNumber("yaw", this::getYaw).withWidget(BuiltInWidgets.kGyro);
        navXtab.addNumber("pitch", this::getPitch).withWidget(BuiltInWidgets.kGyro);
        navXtab.addNumber("roll", this::getRoll).withWidget(BuiltInWidgets.kGyro);
        navXtab.addBoolean("isMagnetometerCalibrated", this::isMagnetometerCalibrated)
                .withWidget(BuiltInWidgets.kBooleanBox);
        navXtab.addNumber("fusedHeading", this::getFusedHeading).withWidget(BuiltInWidgets.kGyro);
        navXtab.addNumber("Absolute Yaw", () -> getAbsoluteRotation().getDegrees())
                .withWidget(BuiltInWidgets.kGyro);
    }


    public double getYaw() {
        return m_gyro.getYaw() + yawOffset;
    }

    /** Yaw offset will be added to yaw (-180, 180) but positive is clockwise */
    public void configureYawOffset(double yawOffset) {
        this.yawOffset = yawOffset;
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
        double invertedYaw = 360.0 - getYaw();
        //get absolute
        double absolute0To360 = (invertedYaw - 180.0) % 360;
        //must substract 180 again
        return Rotation2d.fromDegrees(absolute0To360 - 180.0);
    }

    @Override
    public void periodic() {
        var lastRotation = RobotState.getInstance().getCurrentGyroRotation();
        RobotState.getInstance().setCurrentGyroRotation(getAbsoluteRotation());
        RobotState.getInstance().setLastGyroRotation(lastRotation);
    }

    @Override
    public void onEnable() {
        zeroYaw();
        var startPose = RobotState.getInstance().getStartPose();
        configureYawOffset(startPose.getRotation().getDegrees());
    }

    @Override
    public void onDisable() {
        configureYawOffset(0);
    }
}
