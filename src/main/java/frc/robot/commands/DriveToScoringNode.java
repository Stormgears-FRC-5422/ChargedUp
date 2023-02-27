package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.commands.trajectory.FollowPathCommand;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.DrivetrainBase;

public class DriveToScoringNode extends CommandBase {

    //delegate pathfollowing to command
    private FollowPathCommand m_command;
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
        //get the heading towards the scoring position for smooth driving
        double initialHeading = calcInitialHeading(currentPose.getTranslation(), m_node.scoringPosition.getTranslation());
        var initialPoint = new PathPoint(
                currentPose.getTranslation(),
                Rotation2d.fromRadians(initialHeading),
                currentPose.getRotation());
        //if we want to go blue then come in at a 180 else come in at 0
        var endingPoint = new PathPoint(
                m_node.scoringPosition.getTranslation(),
                (m_node.alliance == DriverStation.Alliance.Blue)? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0),
                m_node.scoringPosition.getRotation());

        var pathToFollow = PathPlanner.generatePath(
                new PathConstraints(maxVel, maxAcc),
                initialPoint, endingPoint
        );
        m_command = new FollowPathCommand(pathToFollow, m_drivetrain);
        System.out.println("Driving to node at " +
                new Translation2d(m_node.translation.getX(), m_node.translation.getY()) +
                " to score: " + m_node.type);
        m_command.initialize();
    }

    @Override
    public void execute() {
        m_command.execute();
    }

    @Override
    public boolean isFinished() {
        return m_command.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        m_command.end(interrupted);
    }

    /** gives desired heading to scoring position in radians */
    private double calcInitialHeading(Translation2d initialTranslation, Translation2d endingTranslation) {
        //Move the ending about the initial
        Translation2d aboutInitialTranslation = endingTranslation.minus(initialTranslation);
        //This is to find the reference angle and the heading in radians
        return Math.atan2(aboutInitialTranslation.getY(), aboutInitialTranslation.getX());
    }
}
