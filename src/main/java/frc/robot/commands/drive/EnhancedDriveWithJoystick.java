package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.DrivetrainBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class EnhancedDriveWithJoystick extends CommandBase {

    private final DrivetrainBase m_drivetrain;

    private final DoubleSupplier txSupplier, tySupplier, omegaSupplier,
                            rotSetpointSupplier;
    private final BooleanSupplier robotRelativeSupplier;
    private final ProfiledPIDController rotController = new ProfiledPIDController(Constants.turnkp, 0, 0,
            new TrapezoidProfile.Constraints(120, 30));

    private boolean percisionMode = false;

    /**
     * Drive command which can control rotation using a setpoint supplier for rotaion control
     * @param txSupplier translation X Vel in WPI coords
     * @param tySupplier translation Y Vel in WPI coords
     * @param omegaSupplier rotation vel negative is clockwise
     * @param rotSetpointSupplier angle the robot should hold
     * @param robotRelativeSupplier
     */
    public EnhancedDriveWithJoystick(DrivetrainBase m_drivetrain,
                                     DoubleSupplier txSupplier, DoubleSupplier tySupplier, DoubleSupplier omegaSupplier,
                                     DoubleSupplier rotSetpointSupplier,
                                     BooleanSupplier robotRelativeSupplier) {
        this.m_drivetrain = m_drivetrain;
        this.txSupplier = txSupplier;
        this.tySupplier = tySupplier;
        this.omegaSupplier = omegaSupplier;
        this.rotSetpointSupplier = rotSetpointSupplier;
        this.robotRelativeSupplier = robotRelativeSupplier;
    }

    @Override
    public void execute() {
        super.execute();
    }
}
