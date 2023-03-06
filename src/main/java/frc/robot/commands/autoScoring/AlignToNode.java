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
import frc.robot.commands.pathFollowing.PathFollowingCommand;
import frc.robot.commands.pathFollowing.Paths;
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
        double initialHeading = Paths.calcHeading(currentPose.getTranslation(), intermediateTranslation);
        // make the rotation part smooth by finding the intermediate rotation
        double distToIntermediate = currentPose.getTranslation().getDistance(intermediateTranslation);
        double totalDist = distToIntermediate + intermediateTranslation.getDistance(alignedPose.getTranslation());
        Rotation2d intermediateRot = currentPose.getRotation().interpolate(scoringPose.getRotation(),
                distToIntermediate / totalDist);
        // make the path from current pose -> intermediate -> aligned
        // still have to drive to scoring position, this makes time for arm movements
        var path = PathPlanner.generatePath(
                new PathConstraints(2, 1.5),
                new PathPoint(currentPose.getTranslation(), new Rotation2d(initialHeading), currentPose.getRotation())
                        .withNextControlLength(distToIntermediate),
                new PathPoint(intermediateTranslation, new Rotation2d(), intermediateRot)
                        .withControlLengths(0, 0),
                new PathPoint(alignedPose.getTranslation(), new Rotation2d(), alignedPose.getRotation())
                        .withControlLengths(0, 0)
        );
        withPath(path);
        super.initialize();
    }
}
