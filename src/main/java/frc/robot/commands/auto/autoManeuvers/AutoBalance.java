package frc.robot.commands.auto.autoManeuvers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.commands.drive.BalanceCommand;
import frc.robot.commands.drive.BalancePitchCommand;
import frc.robot.constants.ShuffleboardConstants;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.drive.DrivetrainBase;

public class AutoBalance extends CommandBase {
    private final DrivetrainBase drivetrain;
    private final NavX navX;

    private static GenericEntry balanceSpeed;
    private static GenericEntry balanceThresh;

    private PIDController rotController = new PIDController(0.02, 0.0, 0.0, 0.02);
    private double rotSetpoint = 0.0;

    public AutoBalance(DrivetrainBase drivetrain, NavX navX) {
        this.drivetrain = drivetrain;
        this.navX = navX;

        balanceSpeed = ShuffleboardConstants.getInstance().preRoundTab
                .add("Balance Speed", 1.0)
                .getEntry();
        balanceThresh = ShuffleboardConstants.getInstance().preRoundTab
                .add("Balance Threshold Degrees", 2.5)
                .getEntry();
    }

    @Override
    public void initialize() {
        // get the closest by using the sign of the cos
        double currentRotationX = RobotState.getInstance().getCurrentPose().getRotation().getCos();
        rotSetpoint = new Rotation2d(currentRotationX, 0.0).getDegrees();
        rotController.setSetpoint(rotSetpoint);
    }

    @Override
    public void execute() {
        double x, theta;

        // make sure to have tolerance for pitch
        double pitchReading = navX.getPitch();
        if (Math.abs(pitchReading) <= balanceThresh.getDouble(2.5))
            pitchReading = 0.0;

        double speed = balanceSpeed.getDouble(1.0);
        if (pitchReading == 0)
            x = 0;
        else
        // move backward when pitch is positive vice versa
            x = -Math.signum(pitchReading) * speed;

        // make sure the robot is straight
        double currYaw = RobotState.getInstance().getCurrentPose().getRotation().getDegrees();
        theta = rotController.calculate(currYaw);

        // drive in robot relative
        drivetrain.drive(new ChassisSpeeds(x, 0, theta), false);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopDrive();
    }
}
