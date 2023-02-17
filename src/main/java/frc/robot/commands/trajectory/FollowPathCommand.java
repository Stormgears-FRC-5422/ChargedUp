package frc.robot.commands.trajectory;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DrivetrainBase;

public class FollowPathCommand extends CommandBase {

    private final PathPlannerTrajectory m_path;
    private final DrivetrainBase m_drivetrain;

    private double startTime, currentTime, endTime;

    public FollowPathCommand(PathPlannerTrajectory path, DrivetrainBase drivetrain) {
        m_path = path;
        m_drivetrain = drivetrain;

        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        startTime = RobotState.getInstance().getTimeSeconds();
        endTime = m_path.getTotalTimeSeconds();
        System.out.println("Path Following Command Starting at: " + startTime);
    }

    @Override
    public void execute() {
        currentTime = RobotState.getInstance().getTimeSeconds() - startTime;
        Trajectory.State goalState = m_path.sample(currentTime);
        var goalPose = goalState.poseMeters;
        var currentPose = RobotState.getInstance().getCurrentPose();
        System.out.println("Distance to goal translation: " + currentPose.getTranslation().getDistance(goalPose.getTranslation()));
        System.out.println("Degrees to holonomic rotation: " + (currentPose.getRotation().getDegrees() - goalPose.getRotation().getDegrees()));
        m_drivetrain.goToTrajectoryState(goalState);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Ending with interrupted: " + interrupted);
        System.out.println("Path Follow Command ended at: " + RobotState.getInstance().getTimeSeconds());
    }

    @Override
    public boolean isFinished() {
        return currentTime >= endTime;
    }
}
