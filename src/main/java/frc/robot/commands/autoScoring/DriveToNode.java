package frc.robot.commands.autoScoring;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
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

import java.util.List;

import static frc.robot.constants.FieldConstants.Grids.ScoringNode;
import static frc.robot.commands.drive.pathFollowing.Paths.calcHeading;

public class DriveToNode extends PathFollowingCommand {

    private final ScoringNode goalNode;

    public DriveToNode(DrivetrainBase drivetrain, ScoringNode goalNode) {
        super(drivetrain);
        this.goalNode = goalNode;
    }

//    @Override
//    public void initialize() {
//        var currentPose = RobotState.getInstance().getCurrentPose();
//        if (goalNode.gridRegion.inRegion(currentPose)) end(true);
//        var scoringPose = goalNode.scoringPosition;
//        // come in from some distance out and if it is red then we have to subtract instead of add
//        boolean isRed = goalNode.alliance == DriverStation.Alliance.Red;
//        double alignedX = scoringPose.getX() + (isRed? -1.0 : 1.0) * Units.inchesToMeters(10);
//        Pose2d alignedPose = new Pose2d(alignedX, scoringPose.getY(), scoringPose.getRotation());
//        // make an intermediate translation to go around possible obstacles in the most simple way
//        Translation2d intermediateTranslation = new Translation2d(alignedX, currentPose.getY());
//        // calculate the direction of travel for going to intermediate pose and then aligned
//        double initialHeading = Paths.calcHeading(currentPose.getTranslation(), intermediateTranslation);
//        double endHeading = Paths.calcHeading(alignedPose.getTranslation(), intermediateTranslation);
//        // use distances to control the heading control lengths make the start heading be stronger for a curve
//        double startToScoringPose = currentPose.getTranslation().getDistance(scoringPose.getTranslation());
//        double intermediateToEnd = alignedPose.getTranslation().getDistance(intermediateTranslation);
//        // make the path from current pose  -> aligned
//        // still have to drive to scoring position, this makes time for arm movements
//        var path = PathPlanner.generatePath(
//                new PathConstraints(1, 0.6),
//                new PathPoint(currentPose.getTranslation(), new Rotation2d(initialHeading), currentPose.getRotation())
//                        .withNextControlLength(startToScoringPose),
//                new PathPoint(alignedPose.getTranslation(), new Rotation2d(endHeading), alignedPose.getRotation())
//                        .withPrevControlLength(intermediateToEnd)
//        );
//        withPath(path);
//        super.initialize();
//    }


    @Override
    public void initialize() {
        Pose2d currentPose = RobotState.getInstance().getCurrentPose();
        double currentVel = RobotState.getInstance().getCurrentLinearVel();

        Pose2d scorePose = goalNode.scoringPosition;
        boolean isRed = goalNode.alliance == DriverStation.Alliance.Red;
        boolean inRegion = goalNode.gridRegion.inRegion(currentPose);

        PathPlannerTrajectory path;
        if (inRegion) {
            path = Paths.generatePathToPose(currentPose, currentVel, scorePose, 1, 1);
        } else {
            double alignedX = scorePose.getX() + (isRed? -1.0 : 1.0) * Units.inchesToMeters(10);

            Translation2d alignedToGrid = new Translation2d(alignedX, currentPose.getY());
            Translation2d alignedToNode = new Translation2d(alignedX, scorePose.getY());

            PathPoint startPoint = new PathPoint(
                    currentPose.getTranslation(),
                    Rotation2d.fromRadians(calcHeading(currentPose.getTranslation(), alignedToGrid)),
                    currentPose.getRotation(), currentVel);

            PathPoint alignToGrids = new PathPoint(
                    alignedToGrid,
                    Rotation2d.fromRadians(calcHeading(alignedToGrid, alignedToNode)),
                    scorePose.getRotation()).withPrevControlLength(0.05);

            PathPoint alignToNode = new PathPoint(
                    alignedToNode,
                    Rotation2d.fromRadians(calcHeading(alignedToNode, scorePose.getTranslation())),
                    scorePose.getRotation()).withPrevControlLength(0.05);

            PathPoint scoringPose = new PathPoint(
                    scorePose.getTranslation(),
                    scorePose.getRotation(),
                    scorePose.getRotation()).withPrevControlLength(Units.inchesToMeters(10));

            path = PathPlanner.generatePath(
                    new PathConstraints(2, 0.5),
                    List.of(startPoint, alignToGrids, alignToNode, scoringPose));
        }

        withPath(path);
        super.initialize();
    }
}
