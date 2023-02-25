package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DrivetrainBase;

public class TrapezoidMoveForward extends CommandBase {

    private final DrivetrainBase m_drivetrain;
    private final TrapezoidProfileCommand m_profileCommand;

    //FIXME: add constants
    private final PIDController m_positionController = new PIDController(1.3, 0, 0);
    private final SimpleMotorFeedforward m_velocityFF = new SimpleMotorFeedforward(0.1, 1, 0.1);

    private double m_totalDistance;
    private final double m_goalDistance;

    public TrapezoidMoveForward(DrivetrainBase drivetrain,
                                double goalDistance, double maxVelocity, double maxAcceleration) {
        this(drivetrain, new TrapezoidProfile(
                new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration),
                new TrapezoidProfile.State(goalDistance, 0)
        ));
    }

    public TrapezoidMoveForward(DrivetrainBase drivetrain, TrapezoidProfile profile) {
        m_drivetrain = drivetrain;
        m_goalDistance = profile.calculate(profile.totalTime()).position;
        m_profileCommand = new TrapezoidProfileCommand(profile, this::goToState, m_drivetrain);
        m_totalDistance = 0;
        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        m_profileCommand.initialize();
        m_totalDistance = 0;
        System.out.println("goal Distance: " + m_goalDistance);
        m_drivetrain.setDriveSpeedScale(1);
    }

    @Override
    public void execute() {
        m_totalDistance += RobotState.getInstance().getDeltaDistanceMeters();
        m_profileCommand.execute();
        SmartDashboard.putNumber("totalDistance", m_totalDistance);
        SmartDashboard.putNumber("current vel", (RobotState.getInstance().getDeltaDistanceMeters()/20.) * 1000);
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
        if (!interrupted)
            m_drivetrain.stopDrive();
        m_profileCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return (m_profileCommand.isFinished()) && (Math.abs(m_totalDistance - m_goalDistance) < Units.inchesToMeters(3));
    }
}
