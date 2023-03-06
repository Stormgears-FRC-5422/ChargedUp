package frc.robot.commands.drive;
;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DrivetrainBase;

import java.util.function.DoubleSupplier;

public class BalancePitchCommand extends CommandBase {

    // tune these
    private final PIDController controller = new PIDController(0.02, 0.1, 0.01);

    private final DrivetrainBase m_drivetrain;
    private final DoubleSupplier pitchSupplier;

    public BalancePitchCommand(DrivetrainBase drivetrain, DoubleSupplier pitchSupplier) {
        m_drivetrain = drivetrain;
        this.pitchSupplier = pitchSupplier;

        controller.setTolerance(2.5);

        addRequirements(m_drivetrain);
    }

    @Override
    public void execute() {
        // test to see if we should negate
        double pidOut = controller.calculate(pitchSupplier.getAsDouble(), 0);
        m_drivetrain.drive(new ChassisSpeeds(pidOut, 0.0, 0.0), true);
    }

    @Override
    public boolean isFinished() {
        return controller.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted)
            m_drivetrain.stopDrive();
    }
}
