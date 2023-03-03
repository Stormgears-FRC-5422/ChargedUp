package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.commands.pathFollowing.FollowPathCommand;
import frc.robot.commands.pathFollowing.Paths;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.drive.DrivetrainBase;

import static frc.robot.constants.Constants.BUMPER_THICKNESS;
import static frc.robot.constants.Constants.ROBOT_LENGTH;

public class DriveToScoringNode extends CommandBase {

    //delegate pathfollowing to command
    private SequentialCommandGroup m_followCommands;
    private final DrivetrainBase m_drivetrain;
    private final FieldConstants.Grids.ScoringNode m_node;

    private final double maxVel, maxAcc;

    public DriveToScoringNode(DrivetrainBase drivetrain, FieldConstants.Grids.ScoringNode node,
                              double maxVel, double maxAcc) {
        m_drivetrain = drivetrain;
        m_node = node;

        this.maxVel = maxVel;
        this.maxAcc = maxAcc;

        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        var currentPose = RobotState.getInstance().getCurrentPose();
        var scoringPose = m_node.scoringPosition;
        //drive in from 1.2 feet out
        double middleX = Units.inchesToMeters(54.05) + (ROBOT_LENGTH / 2.0) + BUMPER_THICKNESS + Units.feetToMeters(1.2);
        if (m_node.alliance == DriverStation.Alliance.Red) middleX = FieldConstants.mirrorXPosition(middleX);
        Pose2d middlePose = new Pose2d(middleX, scoringPose.getY(), scoringPose.getRotation());

        var startPoint = new PathPoint(
                currentPose.getTranslation(),
                new Rotation2d(0),
                currentPose.getRotation())
                .withControlLengths(0, 0);

        var middlePoint = new PathPoint(
                middlePose.getTranslation(),
                new Rotation2d(0),
                middlePose.getRotation())
                .withControlLengths(0, 0);

        var endPoint = new PathPoint(
                scoringPose.getTranslation(),
                new Rotation2d(),
                scoringPose.getRotation())
                .withControlLengths(0, 0);
        var firstPath = PathPlanner.generatePath(
                new PathConstraints(maxVel, maxAcc),
                startPoint, middlePoint
        );

        var lastPath = PathPlanner.generatePath(
                new PathConstraints(0.5, 0.5),
                middlePoint, endPoint
        );

        m_followCommands = new SequentialCommandGroup(
                new FollowPathCommand(firstPath, m_drivetrain),
                new FollowPathCommand(lastPath, m_drivetrain)
        );
    }

    @Override
    public void execute() {
        m_followCommands.execute();
    }

    @Override
    public boolean isFinished() {
        return m_followCommands.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        m_followCommands.end(interrupted);
    }
}
