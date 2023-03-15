package frc.robot.commands.auto.autoScoring;

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
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.DrivetrainBase;

import java.util.List;
import java.util.function.Supplier;

import static frc.robot.constants.FieldConstants.Grids.ScoringNode;
import static frc.robot.commands.drive.pathFollowing.Paths.calcHeading;

public class DriveToNode extends PathFollowingCommand {

    private final Supplier<ScoringNode> goalNodeSupplier;

    public DriveToNode(DrivetrainBase drivetrain, Supplier<ScoringNode> goalNode) {
        super(drivetrain);
        this.goalNodeSupplier = goalNode;
    }

    @Override
    public void initialize() {
        Pose2d currentPose = RobotState.getInstance().getCurrentPose();
        double currentVel = RobotState.getInstance().getCurrentLinearVel();

        var goalNode = goalNodeSupplier.get();
        System.out.println("Driving to node: " + goalNode);
        Pose2d scorePose = goalNode.scoringPosition;
        boolean isRed = goalNode.alliance == DriverStation.Alliance.Red;
        boolean inRegion = goalNode.gridRegion.contains(currentPose);
        
        double distForAligned = Constants.TURN_RADIUS + Constants.BUMPER_THICKNESS + Units.inchesToMeters(7.0);

        PathPlannerTrajectory path;
        if (inRegion) {
            System.out.println("Using quick path to node inside " + goalNode.gridRegion);
            PathPoint startPoint = new PathPoint(
                    currentPose.getTranslation(),
                    Rotation2d.fromRadians(calcHeading(currentPose.getTranslation(), scorePose.getTranslation())),
                    currentPose.getRotation(), currentVel);
                    
            PathPoint endPoint = new PathPoint(
                    scorePose.getTranslation(),
                    scorePose.getRotation(),
                    scorePose.getRotation()).withPrevControlLength(distForAligned);
                    
            path = PathPlanner.generatePath(
                    new PathConstraints(1.0, 1.0),
                    startPoint, endPoint);
        } else {
            double alignedX = scorePose.getX() + (isRed? -1.0 : 1.0) * distForAligned;
            
            Translation2d alignedToNode = new Translation2d(alignedX, scorePose.getY());
            Translation2d alignedToGrid = new Translation2d(alignedX, currentPose.getY());

            double distToAlignedToGrid = currentPose.getTranslation().getDistance(alignedToGrid);
            double distToAlignedToNode = alignedToGrid.getDistance(alignedToNode) + distToAlignedToGrid;

            Rotation2d scoreRotation = scorePose.getRotation();
            Rotation2d firstRotation = currentPose.getRotation()
                    .interpolate(scoreRotation, distToAlignedToGrid / distToAlignedToNode);

            PathPoint startPoint = new PathPoint(
                    currentPose.getTranslation(),
                    Rotation2d.fromRadians(calcHeading(currentPose.getTranslation(), alignedToGrid)),
                    currentPose.getRotation(), currentVel);

            PathPoint alignToGrids = new PathPoint(
                    alignedToGrid,
                    Rotation2d.fromRadians(calcHeading(alignedToGrid, alignedToNode)),
                    firstRotation).withPrevControlLength(0.05);

            PathPoint alignToNode = new PathPoint(
                    alignedToNode,
                    Rotation2d.fromRadians(calcHeading(alignedToNode, scorePose.getTranslation())),
                    scoreRotation).withPrevControlLength(0.05);

            PathPoint scoringPose = new PathPoint(
                    scorePose.getTranslation(),
                    scorePose.getRotation(),
                    scoreRotation).withPrevControlLength(Units.inchesToMeters(10));

            path = PathPlanner.generatePath(
                    new PathConstraints(1.5, 1),
                    List.of(startPoint, alignToGrids, alignToNode, scoringPose));
        }
        withPath(path);
        super.initialize();
    }
}
