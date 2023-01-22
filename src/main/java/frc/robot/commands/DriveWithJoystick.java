package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.subsystems.drive.SDSDrivetrain;

import java.util.function.DoubleSupplier;

public class DriveWithJoystick extends CommandBase {

    private final DrivetrainBase m_drivetrain;
    private final DoubleSupplier txSupplier;
    private final DoubleSupplier tySupplier;
    private final DoubleSupplier rotSupplier;

    /**
     * USE WPILIB COORDINATES X DRIVES FORWARDS, Y DRIVES LEFT
     * using percentage inputs (-1, 1)
     * @param drive
     * @param translationXSupplier
     * @param translationYSupplier
     * @param rotationSupplier
     */
    public DriveWithJoystick(DrivetrainBase drive,
                             DoubleSupplier translationXSupplier,
                             DoubleSupplier translationYSupplier,
                             DoubleSupplier rotationSupplier) {
        this.m_drivetrain = drive;
        this.txSupplier = translationXSupplier;
        this.tySupplier = translationYSupplier;
        this.rotSupplier = rotationSupplier;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        m_drivetrain.percentOutDrive(txSupplier.getAsDouble(), tySupplier.getAsDouble(), rotSupplier.getAsDouble());
    }
}
