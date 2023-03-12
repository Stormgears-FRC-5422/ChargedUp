package frc.robot.commands.drive.pathFollowing;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DrivetrainBase;

public class DriveToPose extends PathFollowingCommand {

    private final Pose2d goalPose;
    private final double maxVel, maxAcc;

    public DriveToPose(DrivetrainBase drivetrain, Pose2d goalPose,
                       double maxVel, double maxAcc) {
        super(drivetrain);

        this.goalPose = goalPose;
        this.maxVel = maxVel;
        this.maxAcc = maxAcc;
    }

    @Override
    public void initialize() {
        var currPose = RobotState.getInstance().getCurrentPose();
        double currVel = RobotState.getInstance().getCurrentLinearVel();
        System.out.println("Drive to pose with goal: " + goalPose);
        withPath(Paths.generatePathToPose(currPose, currVel, goalPose, maxVel, maxAcc));
        super.initialize();
    }
}