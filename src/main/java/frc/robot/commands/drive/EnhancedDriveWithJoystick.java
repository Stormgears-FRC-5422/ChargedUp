package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.DrivetrainBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class EnhancedDriveWithJoystick extends CommandBase {

    private final DrivetrainBase m_drivetrain;

    private final DoubleSupplier txSupplier, tySupplier, omegaSupplier;
    private final BooleanSupplier robotRelativeSupplier, percisionModeSupplier;
    private final ProfiledPIDController rotController =
            new ProfiledPIDController(0.01, 0.0, 0.0,
                new TrapezoidProfile.Constraints(180, 70));
    private final DoubleSupplier robotAngleSupplier;

    private double omegaSpeed;
    private boolean setpointRotationMode = false;
    private String setpointDirection = "NOTHING";

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
        rotController.setTolerance(1.5);

        robotAngleSupplier = () -> (Constants.SubsystemToggles.usePoseEstimator)?
                RobotState.getInstance().getCurrentPose().getRotation().getDegrees() :
                RobotState.getInstance().getCurrentGyroRotation().getDegrees();

        addRequirements(m_drivetrain);
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
        }

        _drive();
    }

    private void _drive() {
        if (percisionModeSupplier.getAsBoolean()) {
            m_drivetrain.setDriveSpeedScale(Constants.kPrecisionSpeedScale);
        } else {
            m_drivetrain.setDriveSpeedScale(Constants.kDriveSpeedScale);
        }
        m_drivetrain.percentOutDrive(
                new ChassisSpeeds(
                        txSupplier.getAsDouble(),
                        tySupplier.getAsDouble(),
                        omegaSpeed),
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
                setpointDirection = "NOTHING";
                break;
        }
        rotController.setGoal(angle);
        setpointRotationMode = true;
    }

    public boolean getFieldOriented() {
        return !robotRelativeSupplier.getAsBoolean();
    }

    public boolean getPercisionMode() {
        return percisionModeSupplier.getAsBoolean();
    }

    public String getSetpointDirection() {
        return setpointDirection;
    }
}
