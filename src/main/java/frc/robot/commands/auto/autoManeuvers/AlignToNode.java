package frc.robot.commands.auto.autoManeuvers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.utils.joysticks.DriveJoystick;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.constants.FieldConstants.Grids.ScoringNode;

public class AlignToNode extends CommandBase {

    private final PIDController yController = new PIDController(0.7, 0.0, 0.01);
    private final PIDController rotController = new PIDController(0.01, 0.0, 0.0);

    private final DrivetrainBase drivetrain;
    private final DoubleSupplier joystickX, joystickY;
    private final Supplier<ScoringNode> nodeSupplier;

    private boolean isRed = false;

    private double xSetpoint, ySetpoint, rotSetpoint;

    private static double minJoystickX = Constants.kPrecisionSpeedScale * 0.25;
    private static double maxJoystickX = Constants.kPrecisionSpeedScale * 1.2;
    private static double maxXDistance = 1.5;

    private static double maxXSpeed = 0.5;
    private static double maxYSpeed = 0.5;
    private static double maxRotSpeed = 0.5;

    public AlignToNode(DrivetrainBase drivetrain, DriveJoystick joystick, Supplier<ScoringNode> nodeSupplier) {
        this.drivetrain = drivetrain;
        joystickX = joystick::getWpiXSpeed;
        joystickY = joystick::getWpiYSpeed;
        this.nodeSupplier = nodeSupplier;

        rotController.enableContinuousInput(-180, 180);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        isRed = RobotState.getInstance().getCurrentAlliance() == DriverStation.Alliance.Red;
        ScoringNode goalNode = nodeSupplier.get();
        Pose2d goalPose = goalNode.scoringPosition;

        xSetpoint = goalPose.getX();
        ySetpoint = goalPose.getY();
        rotSetpoint = goalPose.getRotation().getDegrees();

        yController.setSetpoint(ySetpoint);
        rotController.setSetpoint(rotSetpoint);
    }

    @Override
    public void execute() {
        double x, y, rot;
        Pose2d currentPose = RobotState.getInstance().getCurrentPose();

        double negativeIfRed = isRed? -1.0 : 1.0;
        // should be positive when
        double xError = (negativeIfRed) * (currentPose.getX() - xSetpoint);
        double xScale = Math.abs(xError / maxXDistance);
        double xInput = signedSquare(joystickX.getAsDouble()) * xScale;
        xInput = (negativeIfRed) * MathUtil.clamp(xInput, minJoystickX, maxJoystickX);
        x = xInput;
        // stop the robot if already crashing into the grid
        // and driver in trying to move more into the grid
        if (xError <= -0.02 && xInput <= 0.0)
            x = 0;

        // move the setpoint with input
        double yInput = (negativeIfRed) * signedSquare(joystickY.getAsDouble()) * 0.01;
        ySetpoint += yInput;
        yController.setSetpoint(ySetpoint);
        y = yController.calculate(currentPose.getY());

        rot = rotController.calculate(rotSetpoint);

        x = MathUtil.clamp(x, -maxXSpeed, maxXSpeed);
        y = MathUtil.clamp(y, -maxYSpeed, maxYSpeed);
        rot = MathUtil.clamp(rot, -maxRotSpeed, maxRotSpeed);
        // calculate x and y speeds based on absolute field relativeness
        ChassisSpeeds s = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, 0, currentPose.getRotation());
        drivetrain.percentOutDrive(new ChassisSpeeds(s.vxMetersPerSecond, s.vyMetersPerSecond, rot), false);
    }

    private static double signedSquare(double input) {
        return Math.signum(input) * Math.pow(input, 2);
    }
}
