package frc.robot.commands.autoScoring;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotState;
import frc.robot.commands.drive.pathFollowing.PathFollowingCommand;
import frc.robot.commands.drive.pathFollowing.Paths;
import frc.robot.subsystems.drive.DrivetrainBase;

import static frc.robot.constants.FieldConstants.Grids.ScoringNode;

public class AlignToNode extends PathFollowingCommand {

    private final ScoringNode goalNode;

    public AlignToNode(DrivetrainBase drivetrain, ScoringNode goalNode) {
        super(drivetrain);
        this.goalNode = goalNode;
    }

    @Override
    public void initialize() {
        var currentPose = RobotState.getInstance().getCurrentPose();
        var scoringPose = goalNode.scoringPosition;
        // come in from 1.2 feet out and if it is red then we have to subtract instead of add
        boolean isRed = goalNode.alliance == DriverStation.Alliance.Red;
        double alignedX = scoringPose.getX() + (isRed? -1.0 : 1.0) * Units.feetToMeters(1.2);
        Pose2d alignedPose = new Pose2d(alignedX, scoringPose.getY(), scoringPose.getRotation());
        // make an intermediate translation to go around possible obstacles in the most simple way
        Translation2d intermediateTranslation = new Translation2d(alignedX, currentPose.getY());
        // calculate the direction of travel for going to intermediate pose and then aligned
        double initialHeading = Paths.calcHeading(currentPose.getTranslation(), intermediateTranslation);
        double endHeading = Paths.calcHeading(alignedPose.getTranslation(), intermediateTranslation);
        // use distances to control the heading control lengths make the start heading be stronger for a curve
        double startToScoringPose = currentPose.getTranslation().getDistance(scoringPose.getTranslation());
        double intermediateToEnd = alignedPose.getTranslation().getDistance(intermediateTranslation);
        // make the path from current pose  -> aligned
        // still have to drive to scoring position, this makes time for arm movements
        var path = PathPlanner.generatePath(
                new PathConstraints(2, 1.5),
                new PathPoint(currentPose.getTranslation(), new Rotation2d(initialHeading), currentPose.getRotation())
                        .withNextControlLength(startToScoringPose),
                new PathPoint(alignedPose.getTranslation(), new Rotation2d(endHeading), alignedPose.getRotation())
                        .withPrevControlLength(intermediateToEnd)
        );
        withPath(path);
        super.initialize();
    }
}
