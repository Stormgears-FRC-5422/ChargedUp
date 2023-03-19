package frc.robot.commands.drive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DrivetrainBase;

import java.util.function.DoubleSupplier;

public class BalancePitchCommand extends CommandBase {

    // tune these
    private final PIDController speedController = new PIDController(0.1, 0.0, 0.00);
    private final PIDController rotController = new PIDController(0.1, 0.0, 0.0);

    double holdAngle = 0;
    private final DrivetrainBase m_drivetrain;
    private final DoubleSupplier pitchSupplier;

    public BalancePitchCommand(DrivetrainBase drivetrain, DoubleSupplier pitchSupplier) {
        m_drivetrain = drivetrain;
        this.pitchSupplier = pitchSupplier;

        speedController.setTolerance(2.5);
        rotController.enableContinuousInput(-180, 180);
        rotController.setTolerance(1.5);

        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        double currentDegrees = RobotState.getInstance().getCurrentPose()
                .getRotation().getDegrees();

        if (currentDegrees > 90 || currentDegrees < -90)
            holdAngle = 180;
        else
            holdAngle = 0;
    }

    @Override
    public void execute() {
        System.out.println(pitchSupplier.getAsDouble());
        // test to see if we should negate
        double xSpeed = speedController.calculate(pitchSupplier.getAsDouble(), 0);
        double rotSpeed = rotController.calculate(RobotState.getInstance().getCurrentPose()
                .getRotation().getDegrees(), holdAngle);
        m_drivetrain.drive(new ChassisSpeeds(-xSpeed, 0.0, rotSpeed), false);
    }

    // do we need this
    @Override
    public boolean isFinished() {
        return (speedController.atSetpoint() && rotController.atSetpoint());
    }

    // again???
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.stopDrive();
    }
}
