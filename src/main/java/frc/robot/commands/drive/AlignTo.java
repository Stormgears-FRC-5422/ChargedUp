package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DrivetrainBase;

import java.util.function.DoubleSupplier;

public class AlignTo extends CommandBase {

    private final DrivetrainBase m_drivetrain;
    private final DoubleSupplier m_errorSupplier;

    //error -> pout drive
    private final PIDController controller = new PIDController(0.1, 0, 0);

    /** TODO: what is this error going to be i.e. pixels, yaw, meters? */
    public AlignTo(DoubleSupplier errorSupplier, DrivetrainBase drivetrain) {
        m_drivetrain = drivetrain;
        m_errorSupplier = errorSupplier;

        //FIXME: should be based on error supplier
        controller.setTolerance(5);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        System.out.println("Starting align command at: " + RobotState.getInstance().getTimeSeconds());
        m_drivetrain.stopDrive();
    }

    @Override
    public void execute() {
        double controllerOutput = controller.calculate(0, m_errorSupplier.getAsDouble());

        System.out.println("Raw controller output: " + controllerOutput);

        m_drivetrain.percentOutDrive(
                new ChassisSpeeds(0.0, controllerOutput, 0.0),
                false
        );
    }

    @Override
    public boolean isFinished() {
        return controller.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.stopDrive();
        System.out.println("Align command ended at: " + RobotState.getInstance().getTimeSeconds()
                + " with interrupted: " + interrupted);
    }
}
