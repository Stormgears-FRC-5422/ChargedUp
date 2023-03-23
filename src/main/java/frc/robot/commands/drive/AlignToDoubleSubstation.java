package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.DrivetrainBase;

import java.util.function.DoubleSupplier;

public class AlignToDoubleSubstation extends CommandBase {

    private final DrivetrainBase drivetrain;
    private final DoubleSupplier joystickXSupplier, joystickZSupplier;

    private double rotationSetpoint = 0;
    private double xSetpoint = 0;
    private double ySetpoint = 0;
    private Translation2d target = new Translation2d();
    private Side side;
    private boolean shouldFlip;

    private final PIDController rotController = new PIDController(0.01, 0.0, 0.0);

    private final PIDController yController = new PIDController(0.01, 0.0, 0.0);

    private static final double maxRotationSpeed = 0.5;
    private static final double maxYSpeed = 0.5;
    private static final double maxJoystickInput = 0.3;
    // can go maxJoystickInput at this amount of meters
    private static final double maxDistanceX = 2.0;

    public AlignToDoubleSubstation(DrivetrainBase drivetrain,
                                   DoubleSupplier joystickXSupplier,
                                   DoubleSupplier joystickZSupplier,
                                   Side side) {
        this.drivetrain = drivetrain;
        this.joystickXSupplier = joystickXSupplier;
        this.joystickZSupplier = joystickZSupplier;
        this.side = side;

        rotController.enableContinuousInput(-180.0, 180.0);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.setDriveSpeedScale(Constants.kDriveSpeedScale);
        shouldFlip = RobotState.getInstance().getCurrentAlliance() == DriverStation.Alliance.Red;
        if (shouldFlip)
            rotationSetpoint = 180;

        var substation = FieldConstants.Substations.getDoubleSubstation();
        switch (side) {
            case LEFT:
            default:
                target = substation.leftRegion.getCenter();
                break;
            case RIGHT:
                target = substation.rightRegion.getCenter();
        }
        xSetpoint = target.getX();
        ySetpoint = target.getY();

        rotController.setSetpoint(rotationSetpoint);
        yController.setSetpoint(ySetpoint);
    }

    @Override
    public void execute() {
        double omega = 0, x = 0, y = 0;
        Pose2d currentPose = RobotState.getInstance().getCurrentPose();

        omega += rotController.calculate(currentPose.getRotation().getDegrees());
        double joystickZ = signedSquare(joystickZSupplier.getAsDouble()) * maxJoystickInput;
        omega += joystickZ;

        double xError = (shouldFlip? -1.0 : 1.0) * (xSetpoint - currentPose.getX());
        double xScale = xError / maxDistanceX;
//        x = signedSquare(joystickXSupplier.getAsDouble()) * xScale;

        double yError = ySetpoint - currentPose.getY();
//        y = yController.calculate(yError);

        omega = MathUtil.clamp(omega, -maxRotationSpeed, maxRotationSpeed);
        x = MathUtil.clamp(x, -maxJoystickInput, maxJoystickInput);
        y = MathUtil.clamp(y, -maxYSpeed, maxYSpeed);
        drivetrain.percentOutDrive(new ChassisSpeeds(x, y, omega), true);
    }

    private double signedSquare(double input) {
        return Math.signum(input) * Math.pow(input, 2);
    }

    public enum Side {
        LEFT,
        RIGHT
    }

}