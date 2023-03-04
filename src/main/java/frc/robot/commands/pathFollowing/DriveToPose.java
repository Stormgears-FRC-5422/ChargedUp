package frc.robot.commands.pathFollowing;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DrivetrainBase;

public class DriveToPose extends CommandBase {

    private Pose2d startPose;
    private final Pose2d goalPose;
    private final DrivetrainBase m_drivetrain;
    private final double maxVel, maxAcc;

    private FollowPathCommand m_followCommand;

    public DriveToPose(Pose2d startPose, Pose2d goalPose, DrivetrainBase drivetrain,
                       double maxVel, double maxAcc) {
        this.startPose = startPose;
        this.goalPose = goalPose;
        m_drivetrain = drivetrain;
        this.maxVel = maxVel;
        this.maxAcc = maxAcc;

        addRequirements(m_drivetrain);
    }

    public DriveToPose(Pose2d goalPose, DrivetrainBase drivetrain,
                       double maxVel, double maxAcc) {
        this(null, goalPose, drivetrain, maxVel, maxAcc);
    }

    @Override
    public void initialize() {
        if (startPose == null) startPose = RobotState.getInstance().getCurrentPose();
        var path = Paths.getPathToPose(startPose, goalPose, maxVel, maxAcc);
        m_followCommand = new FollowPathCommand(path, m_drivetrain);
    }

    @Override
    public void execute() {
        m_followCommand.execute();
    }

    @Override
    public boolean isFinished() {
        return m_followCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        m_followCommand.end(interrupted);
    }
}
