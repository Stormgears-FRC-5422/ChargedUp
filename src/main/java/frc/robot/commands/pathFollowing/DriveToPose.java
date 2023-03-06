package frc.robot.commands.pathFollowing;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DrivetrainBase;

public class DriveToPose extends PathFollowingCommand {

    private final Pose2d m_goalPose;
    private final double maxVel, maxAcc;

    public DriveToPose(DrivetrainBase drivetrain, Pose2d goalPose,
                       double maxVel, double maxAcc) {
        super(drivetrain);

        m_goalPose = goalPose;
        this.maxVel = maxVel;
        this.maxAcc = maxAcc;
    }

    @Override
    public void initialize() {
        var currentPose = RobotState.getInstance().getCurrentPose();
        var path = Paths.getPathToPose(currentPose, m_goalPose, maxVel, maxAcc);
        System.out.println("Drive to pose with goal: " + m_goalPose);
        withPath(path);
        super.initialize();
    }
}