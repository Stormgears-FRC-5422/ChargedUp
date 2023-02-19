package frc.robot.commands.trajectory;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.RobotState;

import java.util.List;

public final class Paths {
    public static PathWithName straightPath = new PathWithName("Straight", PathPlanner.loadPath("Straight Path", 1, 0.5));
    public static PathWithName straight180Path = new PathWithName("Straight with 180", PathPlanner.loadPath("180 while forward", 1, 0.5));
    public static PathWithName tPath = new PathWithName("T", PathPlanner.loadPath("TPath", 3, 1));
    public static PathWithName circularPath = new PathWithName("Circular", PathPlanner.loadPath("Circular", 1, 0.5));
    public static PathWithName Auto1 = new PathWithName("Auto1 ?? (caution!)", PathPlanner.loadPath("Auto1", 1, 0.5));
    public static PathWithName testPath = new PathWithName("Test ??", PathPlanner.loadPath("Test", 1, 0.5));
    public static PathWithName diagonalPath = getPathToPose("Diagonal",
            new Pose2d(2.5, 1.5, Rotation2d.fromDegrees(-90)),
            2, 3);

    public static List<PathWithName> listOfPaths = List.of(
            straightPath, straight180Path, tPath,
            circularPath, Auto1, testPath, diagonalPath);

    public static PathWithName getPathToPose(String description, Pose2d startPose, Pose2d endPose, double maxVel, double maxAcc) {
        return new PathWithName(
                description,
                PathPlanner.generatePath(
                    new PathConstraints(maxVel, maxAcc),
                    new PathPoint(startPose.getTranslation(), new Rotation2d(), startPose.getRotation()),
                    new PathPoint(endPose.getTranslation(), new Rotation2d(), endPose.getRotation()))
        );
    }

    public static PathWithName getPathToPose(String description, Pose2d endPose, double maxVel, double maxAcc) {
        return getPathToPose(description, new Pose2d(), endPose, maxVel, maxAcc);
    }

    public static PathWithName getPathFromRobotPose(String description, Pose2d endPoseFromRobotPose, double maxVel, double maxAcc) {
        var currentPose = RobotState.getInstance().getCurrentPose();
        Transform2d transform = new Transform2d(currentPose.getTranslation(), currentPose.getRotation());
        return getPathToPose(
                description,
                currentPose,
                endPoseFromRobotPose.transformBy(transform),
                maxVel, maxAcc
        );
    }

    public static class PathWithName {
        public String name;
        public PathPlannerTrajectory path;

        public PathWithName(String name, PathPlannerTrajectory path) {
            this.name = name;
            this.path = path;
        }
    }

    public static class AutoPath extends PathWithName {
        public Pose2d startPose;

        public AutoPath(String name, PathPlannerTrajectory path) {
            super(name, path);
            startPose = path.getInitialHolonomicPose();
        }
    }
}
