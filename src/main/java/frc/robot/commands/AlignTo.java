package frc.robot.commands;

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
    //output can't change by more than 20% in one second 20%/s is max change in velocity (acceleration)
    private final SlewRateLimiter velLimiter = new SlewRateLimiter(0.2);

    private final ShuffleboardTab tab = Shuffleboard.getTab("Align to command");
    private final GenericEntry limitedOutput = tab.add("Limited Output", 0.0).withWidget(BuiltInWidgets.kGraph).getEntry();

    /** TODO: what is this error going to be i.e. pixels, yaw, meters? */
    public AlignTo(DoubleSupplier errorSupplier, DrivetrainBase drivetrain) {
        m_drivetrain = drivetrain;
        m_errorSupplier = errorSupplier;

        //FIXME: should be based on error supplier
        controller.setTolerance(5, 10);
        tab.addBoolean("Are we there?", controller::atSetpoint);
        tab.add("Controller", controller).withWidget(BuiltInWidgets.kPIDController);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        System.out.println("Starting align command at: " + RobotState.getInstance().getTimeSeconds());
        m_drivetrain.stopDrive();
        velLimiter.reset(0);
    }

    @Override
    public void execute() {
        double controllerOutput = controller.calculate(0, m_errorSupplier.getAsDouble());
        double limitedControllerOutput = velLimiter.calculate(controllerOutput);

        System.out.println("Raw controller output: " + controllerOutput);
        System.out.println("Limited controller output: " + limitedControllerOutput);
        limitedOutput.setDouble(limitedControllerOutput);

        m_drivetrain.percentOutDrive(
                new ChassisSpeeds(0.0, limitedControllerOutput, 0.0),
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
