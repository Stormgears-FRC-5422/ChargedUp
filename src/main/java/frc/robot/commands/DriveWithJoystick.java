package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DrivetrainBase;

import java.util.function.DoubleSupplier;

public class DriveWithJoystick extends CommandBase {

    private final DrivetrainBase m_drivetrain;
    private final DoubleSupplier txSupplier;
    private final DoubleSupplier tySupplier;
    private final DoubleSupplier rotSupplier;

    public DriveWithJoystick(DrivetrainBase drive, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier) {
        this.m_drivetrain = drive;
        this.txSupplier = translationXSupplier;
        this.tySupplier = translationYSupplier;
        this.rotSupplier = rotationSupplier;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                txSupplier.getAsDouble(),
                tySupplier.getAsDouble(),
                rotSupplier.getAsDouble(),
                m_drivetrain.getGyroscopeRotation()
                );

        m_drivetrain.drive(speeds);
    }
}
