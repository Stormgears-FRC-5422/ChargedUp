package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.constants.Constants;
import frc.robot.constants.ShuffleboardConstants;
import frc.robot.subsystems.drive.DrivetrainBase;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class EnhancedDriveWithJoystick extends CommandBase {

    private final DrivetrainBase m_drivetrain;

    private final DoubleSupplier txSupplier, tySupplier, omegaSupplier;

    // FIXME: tune these slew rates
    private final SlewRateLimiter txLimiter = new SlewRateLimiter(2);
    private final SlewRateLimiter tyLimiter = new SlewRateLimiter(2);
    private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(2);

    private final BooleanSupplier robotRelativeSupplier, percisionModeSupplier;
    private final ProfiledPIDController rotController =
            new ProfiledPIDController(0.01, 0.0, 0.0,
                new TrapezoidProfile.Constraints(180, 70));
    private final DoubleSupplier robotAngleSupplier;

    private double omegaSpeed;
    private boolean setpointRotationMode = false;
    private String setpointDirection = "NONE";

    public EnhancedDriveWithJoystick(DrivetrainBase drivetrain,
                                     DoubleSupplier txSupplier, DoubleSupplier tySupplier, DoubleSupplier omegaSupplier,
                                     BooleanSupplier robotRelativeSupplier, BooleanSupplier percisionModeSupplier) {
        m_drivetrain = drivetrain;
        this.txSupplier = txSupplier;
        this.tySupplier = tySupplier;
        this.omegaSupplier = omegaSupplier;
        this.robotRelativeSupplier = robotRelativeSupplier;
        this.percisionModeSupplier = percisionModeSupplier;

        rotController.enableContinuousInput(-180.0, 180.0);
        rotController.setTolerance(0.5);

        robotAngleSupplier = () -> (Constants.Toggles.usePoseEstimator)?
                RobotState.getInstance().getCurrentPose().getRotation().getDegrees() :
                RobotState.getInstance().getCurrentGyroData().getDegrees();

        ShuffleboardConstants.getInstance().driverTab
                .addBoolean("Robot Relative", robotRelativeSupplier)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(0, 2).withSize(1, 1);

        ShuffleboardConstants.getInstance().driverTab
                .addBoolean("Percision Mode", percisionModeSupplier)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(1, 2).withSize(1, 1);

        ShuffleboardConstants.getInstance().driverTab
                .addString("Setpoint", () -> setpointDirection)
                .withProperties(Map.of("Label position", "HIDDEN"))
                .withPosition(2, 2).withSize(1, 1);

        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        txLimiter.reset(txSupplier.getAsDouble());
        tyLimiter.reset(tySupplier.getAsDouble());
        omegaLimiter.reset(omegaSupplier.getAsDouble());
        setpointRotationMode = false;
        System.out.println("Drive command starting!");
    }

    @Override
    public void execute() {
        double currentAngle = robotAngleSupplier.getAsDouble();

        if (setpointRotationMode)
            setpointRotationMode = !(Math.abs(omegaSupplier.getAsDouble()) >= 0.2);
        if (setpointRotationMode && !rotController.atGoal()) {
            omegaSpeed = rotController.calculate(currentAngle);
            System.out.println("USING setpoint rotation mode!");
            System.out.println(omegaSpeed);
        } else {
            omegaSpeed = omegaSupplier.getAsDouble();
            // FIXME: do I have to make the vel negative?
            rotController.reset(currentAngle, RobotState.getInstance().getCurrentDegPerSecVel());
            setpointRotationMode = false;
            setpointDirection = "NONE";
        }

        _drive();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Drive command ended!");
    }

    private void _drive() {
        if (percisionModeSupplier.getAsBoolean())
            m_drivetrain.setDriveSpeedScale(Constants.kPrecisionSpeedScale);
        else
            m_drivetrain.setDriveSpeedScale(Constants.kDriveSpeedScale);

        m_drivetrain.percentOutDrive(
                new ChassisSpeeds(
                        txLimiter.calculate(txSupplier.getAsDouble()),
                        tyLimiter.calculate(tySupplier.getAsDouble()),
                        (setpointRotationMode)? omegaSpeed : omegaLimiter.calculate(omegaSpeed)),
                !robotRelativeSupplier.getAsBoolean()
        );
    }

    public void setSetPoint(double angle) {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            angle += (angle < 180.0) ? 180.0 : -180.0;
        }
        switch ((int) angle) {
            case 0:
                setpointDirection = "FORWARD";
                break;
            case 90:
                setpointDirection = "LEFT";
                break;
            case 180:
            case -180:
                setpointDirection = "BACKWARD";
                break;
            case -90:
                setpointDirection = "RIGHT";
                break;
            default:
                setpointDirection = "NONE";
                break;
        }
        rotController.setGoal(angle);
        setpointRotationMode = true;
    }
}