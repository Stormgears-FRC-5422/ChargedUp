package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.subsystems.drive.PoseEstimator;

public class TrapezoidMoveForward extends CommandBase {

    private final DrivetrainBase m_drivetrain;
    private final PoseEstimator m_poseEstimator;
    private final TrapezoidProfileCommand m_profileCommand;

    //FIXME: add constants
    private final PIDController m_positionController = new PIDController(1, 0, 0);
    private final SimpleMotorFeedforward m_velocityFF = new SimpleMotorFeedforward(0, 1, 0);

    private double m_totalDistance;

    public TrapezoidMoveForward(DrivetrainBase drivetrain, PoseEstimator poseEstimator,
                                double goalDistance, double maxVelocity, double maxAcceleration) {
        m_drivetrain = drivetrain;
        m_poseEstimator = poseEstimator;

        TrapezoidProfile profile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration),
                new TrapezoidProfile.State(goalDistance, 0)
        );
        m_profileCommand = new TrapezoidProfileCommand(profile, this::goToState, m_drivetrain);

        m_totalDistance = 0;

        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        m_profileCommand.initialize();
        m_totalDistance = 0;
        m_drivetrain.setDriveSpeedScale(1);
    }

    @Override
    public void execute() {
        m_totalDistance += m_poseEstimator.getDeltaDistance();
        m_profileCommand.execute();

        SmartDashboard.putNumber("totalDistance", m_totalDistance);
    }

    private void goToState(TrapezoidProfile.State state) {
        double pidValue = m_positionController.calculate(m_totalDistance, state.position);
        double ffValue = m_velocityFF.calculate(state.velocity);

        double finalXVel = pidValue + ffValue;

        SmartDashboard.putNumber("reference position", state.position);
        SmartDashboard.putNumber("reference velocity", state.velocity);
        SmartDashboard.putNumber("output", finalXVel);

        m_drivetrain.drive(new ChassisSpeeds(finalXVel, 0, 0), false);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Trapezoid Move Forward Command done!");
        m_drivetrain.stopDrive();
        m_profileCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return m_profileCommand.isFinished();
    }
}
