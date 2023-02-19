package frc.robot.commands.trajectory;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DrivetrainBase;

import java.util.function.Supplier;

public class FollowTrajectoryCommand extends CommandBase {

    private Supplier<Trajectory> m_trajectorySupplier;
    private Trajectory m_trajectory;
    private DrivetrainBase m_drivetrain;
    private SwerveControllerCommand m_swerveController;
    private boolean m_relativeToCurrentPosition;
    private double startTime;
    private double currentTime;
    private double totalTime;


    public FollowTrajectoryCommand(Supplier<Trajectory> trajectorySupplier, DrivetrainBase drivetrain) {
        m_trajectorySupplier = trajectorySupplier;
        m_drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        m_trajectory = m_trajectorySupplier.get();

        startTime = RobotState.getInstance().getTimeSeconds();
        totalTime = m_trajectory.getTotalTimeSeconds();
//        System.out.println("Trajectory start state: " + m_trajectory.sample(0));
//        System.out.println("Trajectory end state: " + m_trajectory.sample(totalTime));
        System.out.println("Trajectory to be followed (transformed): " + m_trajectory);
        System.out.println("Trajectory Follow Command starting at: " + startTime);
        System.out.println("Robot Pose at Start: " + RobotState.getInstance().getCurrentPose());
    }

    @Override
    public void execute() {
        currentTime = RobotState.getInstance().getTimeSeconds() - startTime;
        var goalState = m_trajectory.sample(currentTime);
        m_drivetrain.goToTrajectoryState(goalState);
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted)
            m_drivetrain.stopDrive();
        System.out.println("Robot Pose at end: " + RobotState.getInstance().getCurrentPose());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
