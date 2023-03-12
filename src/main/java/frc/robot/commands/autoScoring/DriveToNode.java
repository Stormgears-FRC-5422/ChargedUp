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
