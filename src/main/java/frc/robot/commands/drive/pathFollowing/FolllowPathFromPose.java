package frc.robot.commands.drive.pathFollowing;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DrivetrainBase;

public class FolllowPathFromPose extends PathFollowingCommand {

    private PathPlannerTrajectory m_path;

    public FolllowPathFromPose(DrivetrainBase drivetrain, PathPlannerTrajectory path) {
        super(drivetrain);
        m_path = path;
    }

    @Override
    public void initialize() {
        var currentPose = RobotState.getInstance().getCurrentPose();
        m_path = (PathPlannerTrajectory) m_path.transformBy(new Transform2d(new Pose2d(), currentPose));
        withPath(m_path);
        super.initialize();
    }
}
