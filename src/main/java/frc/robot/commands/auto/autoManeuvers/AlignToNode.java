package frc.robot.commands.auto.autoManeuvers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ShuffleboardConstants;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.utils.joysticks.DriveJoystick;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.constants.FieldConstants.Grids.ScoringNode;

public class AlignToNode extends CommandBase {

    private final PIDController xController = new PIDController(0.6, 0.0, 0.0);
    private final PIDController yController = new PIDController(0.7, 0.0, 0.0);
    private final PIDController rotController = new PIDController(0.01, 0.0, 0.0);

    private final DrivetrainBase drivetrain;
    private final DoubleSupplier joystickX, joystickY, joystickRot;
    private final Supplier<ScoringNode> nodeSupplier;

    private boolean isRed = false;
    private double negativeIfRed;

    private double xSetpoint, ySetpoint, rotSetpoint;
    private double maxXSetpoint, minXSetpoint;

    // for an output of 1 by joystick the setpoint will move by this amount
    private static double maxSetpointVel = 0.005;
    private static double maxXSpeed = Constants.kPrecisionSpeedScale;
    private static double maxYSpeed = Constants.kPrecisionSpeedScale;
    private static double maxRotSpeed = 0.5;

    public AlignToNode(DrivetrainBase drivetrain, DriveJoystick joystick, Supplier<ScoringNode> nodeSupplier) {
        this.drivetrain = drivetrain;
        joystickX = joystick::getWpiXSpeed;
        joystickY = joystick::getWpiYSpeed;
        joystickRot = joystick::getOmegaSpeed;
        this.nodeSupplier = nodeSupplier;

        xController.setTolerance(Units.inchesToMeters(8.0));
        yController.setTolerance(Units.inchesToMeters(5.0));
        rotController.enableContinuousInput(-180, 180);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.setDriveSpeedScale(1.0);
        isRed = RobotState.getInstance().getCurrentAlliance() == DriverStation.Alliance.Red;
        negativeIfRed = isRed? -1.0 : 1.0;
        ScoringNode goalNode = nodeSupplier.get();
        Pose2d goalPose = goalNode.scoringPosition;
        Pose2d currentPose = RobotState.getInstance().getCurrentPose();

        xSetpoint = goalPose.getX() + (negativeIfRed * Units.inchesToMeters(10.0));
        // if within a foot then don't go back
        if (Math.abs(goalPose.getX() - currentPose.getX()) <= Math.abs(goalPose.getX() - xSetpoint))
            xSetpoint = currentPose.getX();
        // limit x setpoint so we don't drive into wall
        if (isRed) {
            maxXSetpoint = goalPose.getX() + Units.inchesToMeters(1.0);
            minXSetpoint = FieldConstants.FIELD_LENGTH / 2.0;
        } else {
            maxXSetpoint = FieldConstants.FIELD_LENGTH / 2.0;
            minXSetpoint = goalPose.getX() - Units.inchesToMeters(1.0);
        }
        ySetpoint = goalPose.getY();
        rotSetpoint = goalPose.getRotation().getDegrees();

        yController.setSetpoint(ySetpoint);
        rotController.setSetpoint(rotSetpoint);

        ShuffleboardConstants.getInstance().alignToNodeIndicator.setBoolean(true);
    }

    @Override
    public void execute() {
        double x = 0, y = 0, rot = 0;
        Pose2d currentPose = RobotState.getInstance().getCurrentPose();

        // move the setpoint with input if close (minimize driver error)
//        if (xController.atSetpoint()) {
//            double xInput = (negativeIfRed) * signedSquare(joystickX.getAsDouble()) * maxSetpointVel;
//            xSetpoint += xInput;
//            xSetpoint = MathUtil.clamp(xSetpoint, minXSetpoint, maxXSetpoint);
//            xController.setSetpoint(xSetpoint);
//        }
//        x = xController.calculate(currentPose.getX());
        x = (negativeIfRed) * signedSquare(joystickX.getAsDouble()) * maxXSpeed;

        // same idea as x movement
        if (yController.atSetpoint()) {
            double yInput = (negativeIfRed) * signedSquare(joystickY.getAsDouble()) * maxSetpointVel;
            ySetpoint += yInput;
            yController.setSetpoint(ySetpoint);
        }
        // move to x setpoint first before moving to y
//        if (xController.atSetpoint())
            y = yController.calculate(currentPose.getY());

        rot = rotController.calculate(currentPose.getRotation().getDegrees());
        rot += signedSquare(joystickRot.getAsDouble()) * maxRotSpeed;

        x = MathUtil.clamp(x, -maxXSpeed, maxXSpeed);
        y = MathUtil.clamp(y, -maxYSpeed, maxYSpeed);
        rot = MathUtil.clamp(rot, -maxRotSpeed, maxRotSpeed);
        // calculate x and y speeds based on absolute field relativeness
        ChassisSpeeds s = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, currentPose.getRotation());
        drivetrain.percentOutDrive(s, false);
    }

    private static double signedSquare(double input) {
        return Math.signum(input) * Math.pow(input, 2);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        ShuffleboardConstants.getInstance().alignToNodeIndicator.setBoolean(false);
    }
}
