package frc.robot.commands.trajectory;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
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
        System.out.println("Pose at start: " + RobotState.getInstance().getCurrentPose());
    }

    @Override
    public void execute() {
        currentTime = RobotState.getInstance().getTimeSeconds() - startTime;
        var goalState = (PathPlannerTrajectory.PathPlannerState) m_path.sample(currentTime);
        var currentPose = RobotState.getInstance().getCurrentPose();
        //Path Planner states are different to trajectory states
        var goalPose = new Pose2d(
                goalState.poseMeters.getX(),
                goalState.poseMeters.getY(),
                goalState.holonomicRotation);

        System.out.println("Goal Pose: " + goalPose);
        System.out.println("Current Pose: " + currentPose);

        System.out.println("Distance to goal translation: " +
                currentPose.getTranslation().getDistance(goalPose.getTranslation()));
        System.out.println("Degrees to holonomic rotation: " +
                (currentPose.getRotation().getDegrees() - goalPose.getRotation().getDegrees()));

        m_drivetrain.goToPPTrajectoryState(goalState);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Ending with interrupted: " + interrupted);
        System.out.println("Path Follow Command ended at: " + RobotState.getInstance().getTimeSeconds());
        System.out.println("Pose at End: " + RobotState.getInstance().getCurrentPose());
    }

    @Override
    public boolean isFinished() {
        return currentTime >= endTime;
    }
}
