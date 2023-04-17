package frc.robot.commands.drive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DrivetrainBase;

import java.util.function.DoubleSupplier;

public class BalancePitchCommand extends CommandBase {

    // tune these
    private final PIDController speedController = new PIDController(0.02, 0.0, 0.00);
    private final PIDController rotController = new PIDController(0.02, 0.0, 0.0);

    double holdAngle = 0;
    private final DrivetrainBase drivetrain;
    private final DoubleSupplier pitchSupplier;

    public BalancePitchCommand(DrivetrainBase drivetrain, DoubleSupplier pitchSupplier) {
        this.drivetrain = drivetrain;
        this.pitchSupplier = pitchSupplier;

        rotController.enableContinuousInput(-180, 180);
        rotController.setTolerance(1.5);

        speedController.setSetpoint(0.0);
        speedController.setTolerance(2.0);

        addRequirements(this.drivetrain);
    }

    @Override
    public void initialize() {
        Rotation2d current = RobotState.getInstance().getCurrentPose()
                .getRotation();

        holdAngle = new Rotation2d(current.getCos(), 0).getDegrees();
        rotController.setSetpoint(holdAngle);

        drivetrain.setDriveSpeedScale(0.5);
    }

    @Override
    public void execute() {
        System.out.println(pitchSupplier.getAsDouble());
        // test to see if we should negate
        double xSpeed = 0;
        if (!speedController.atSetpoint())
            xSpeed = -speedController.calculate(pitchSupplier.getAsDouble());
        // hold a straight angle
        double rotSpeed = rotController.calculate(
                RobotState.getInstance().getCurrentPose().getRotation().getDegrees());

        drivetrain.percentOutDrive(new ChassisSpeeds(xSpeed, 0.0, rotSpeed), false);
    }

    // do we need this
    @Override
    public boolean isFinished() {
        return false;
    }

    // again???
    @Override
    public void end(boolean interrupted) {
        drivetrain.stopDrive();
    }
}