package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DrivetrainBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveWithJoystick extends CommandBase {

    private final DrivetrainBase m_drivetrain;
    private final DoubleSupplier txSupplier;
    private final DoubleSupplier tySupplier;
    private final DoubleSupplier rotSupplier;
    private final BooleanSupplier precisionModeSupplier;

    private boolean useFieldRelative = false;
    private double gyroOffset = 0.0;

    /**
     * USE WPILIB COORDINATES X DRIVES FORWARDS, Y DRIVES LEFT
     * using percentage inputs (-1, 1)
     * @param drive
     * @param translationXSupplier
     * @param translationYSupplier
     * @param rotationSupplier
     * @param precisionModeSupplier
     */
    public DriveWithJoystick(DrivetrainBase drive,
                             DoubleSupplier translationXSupplier,
                             DoubleSupplier translationYSupplier,
                             DoubleSupplier rotationSupplier,
                             BooleanSupplier precisionModeSupplier) {

        this.m_drivetrain = drive;
        this.txSupplier = translationXSupplier;
        this.tySupplier = translationYSupplier;
        this.rotSupplier = rotationSupplier;
        this.precisionModeSupplier = precisionModeSupplier;

        addRequirements(drive);
    }

    public void toggleFieldRelative() {
        useFieldRelative = !useFieldRelative;
        System.out.println("Field Relative mode toggled: " + useFieldRelative);
    }
    @Override
    public void execute() {
//        SmartDashboard.putNumber("wpi X:", txSupplier.getAsDouble());
//        SmartDashboard.putNumber("wpi y:", tySupplier.getAsDouble());
//        SmartDashboard.putNumber("wpi z:", rotSupplier.getAsDouble());
//        SmartDashboard.putBoolean("fieldOriented", useFieldRelative);
        if (precisionModeSupplier.getAsBoolean() == true) {
            m_drivetrain.setDriveSpeedScale(Constants.kPrecisionSpeedScale);
        } else {
            m_drivetrain.setDriveSpeedScale(Constants.kDriveSpeedScale);
        }

        m_drivetrain.percentOutDrive(
                new ChassisSpeeds(
                        txSupplier.getAsDouble(),
                        tySupplier.getAsDouble(),
                        rotSupplier.getAsDouble()),
                useFieldRelative
        );

        SmartDashboard.putBoolean("fieldRelative", useFieldRelative);
    }
}
