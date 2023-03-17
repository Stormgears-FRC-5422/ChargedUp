package frc.robot.commands.drive.pathFollowing;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DrivetrainBase;

public class DriveToPose extends PathFollowingCommand {

    private Pose2d goalPose = new Pose2d();
    private final double maxVel, maxAcc;

    public DriveToPose(DrivetrainBase drivetrain, Pose2d goalPose,
                       double maxVel, double maxAcc) {
        super(drivetrain);

        this.goalPose = goalPose;
        this.maxVel = maxVel;
        this.maxAcc = maxAcc;
    }

    public DriveToPose(DrivetrainBase drivetrain, double maxVel, double maxAcc) {
        super(drivetrain);

        this.maxVel = maxVel;
        this.maxAcc = maxAcc;
    }

    @Override
    public void initialize() {
        var currPose = RobotState.getInstance().getCurrentPose();
        double currVel = RobotState.getInstance().getCurrentLinearVel();
        addPath(Paths.getPathFromPose(currPose, currVel, goalPose, maxVel, maxAcc));
        super.initialize();
    }

    public void setGoalPose(Pose2d goalPose) {
        this.goalPose = goalPose;
    }
}