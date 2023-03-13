package frc.robot.commands.drive.pathFollowing;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotState;

import java.util.List;

import static com.pathplanner.lib.PathPlanner.*;

public final class Paths {
//    public static final PathPlannerTrajectory straightPath = loadPath("Straight Path", 1, 0.5);
//    public static final PathPlannerTrajectory straight180Path = loadPath("180 while forward", 1, 0.5);
//    public static final PathPlannerTrajectory tPath = loadPath("TPath", 3, 1);
//    public static final PathPlannerTrajectory circularPath = loadPath("Circular", 1, 0.5);
//    public static PathPlannerTrajectory Auto1 = loadPath("Auto1", 1, 0.5);
//    public static final PathPlannerTrajectory testPath = loadPath("Test", 1, 0.5);
//    public static final PathPlannerTrajectory doubleConeChargingStationPath = loadPath("Double Cone Charging Station", 1, 0.5);
//    public static final PathPlannerTrajectory diagonalPath = getPathToPose(
//            new Pose2d(2.5, 1.5, Rotation2d.fromDegrees(-90)),
//            2, 3);
//
//    public static final PathPlannerTrajectory fivePath = PathPlanner.loadPath("5", 2, 3);
//    public static final PathPlannerTrajectory fourPath = PathPlanner.loadPath("4", 2, 3);
//    public static final PathPlannerTrajectory twoPath = PathPlanner.loadPath("2", 2, 3);
//
//    static final PathPlannerTrajectory fiveToFourTransition =
//            getPathToPose(
//                new Pose2d(
//                        fivePath.getEndState().poseMeters.getTranslation(),
//                        fivePath.getEndState().holonomicRotation),
//                fourPath.getInitialHolonomicPose(),
//                0.7, 0.5
//            );
//
//    static final PathPlannerTrajectory fourToTwoTransition =
//            getPathToPose(
//                new Pose2d(
//                        fourPath.getEndState().poseMeters.getTranslation(),
//                        fourPath.getEndState().holonomicRotation),
//                twoPath.getInitialHolonomicPose(),
//                0.7, 0.5
//            );
//
//    static final PathPlannerTrajectory twoToTwoTransition =
//            getPathToPose(
//                new Pose2d(
//                        twoPath.getEndState().poseMeters.getTranslation(),
//                        twoPath.getEndState().holonomicRotation),
//                twoPath.getInitialHolonomicPose(),
//                0.7, 0.5
//            );
//
//    public static final List<PathWithName> listOfPaths = List.of(
//            new PathWithName("Straight", straightPath),
//            new PathWithName("Straight 180", straight180Path),
//            new PathWithName("T", tPath),
//            new PathWithName("Circular", circularPath),
//            new PathWithName("Test", testPath),
//            new PathWithName("Diagonal", diagonalPath),
//            new PathWithName("Double Cone Charging Station", doubleConeChargingStationPath));

    public static PathPlannerTrajectory getPathToPose(Pose2d startPose, Pose2d endPose, double maxVel, double maxAcc) {
        double firstHeading = calcHeading(startPose.getTranslation(), endPose.getTranslation());
        double lastHeading = calcHeading(endPose.getTranslation(), startPose.getTranslation());
        return generatePath(
                    new PathConstraints(maxVel, maxAcc),
                    new PathPoint(startPose.getTranslation(), Rotation2d.fromRadians(firstHeading), startPose.getRotation())
                            .withControlLengths(0.01, 0.01),
                    new PathPoint(endPose.getTranslation(), Rotation2d.fromRadians(lastHeading), endPose.getRotation())
                            .withControlLengths(0.01, 0.01));
    }

    public static PathPlannerTrajectory getPathToPose(Pose2d endPose, double maxVel, double maxAcc) {
        return getPathToPose(new Pose2d(), endPose, maxVel, maxAcc);
    }

    public static PathPlannerTrajectory getPathAbsoluteToCurrentPose(Pose2d endFieldPose, double maxVel, double maxAcc) {
        Pose2d currentPose = RobotState.getInstance().getCurrentPose();
        return getPathToPose(currentPose, endFieldPose, maxVel, maxAcc);
    }


    /** get path to a pose from currentPosition */
    public static PathPlannerTrajectory getPathFromPose(Pose2d currPose, double currVel, Pose2d goalPose, double maxVel, double maxAcc) {
        PathPoint startPoint = new PathPoint(
                currPose.getTranslation(),
                Rotation2d.fromRadians(calcHeading(currPose.getTranslation(), goalPose.getTranslation())),
                currPose.getRotation(), currVel);

        PathPoint endPoint = new PathPoint(
                goalPose.getTranslation(),
                Rotation2d.fromRadians(calcHeading(currPose.getTranslation(), goalPose.getTranslation())),
                goalPose.getRotation());

        return PathPlanner.generatePath(
                new PathConstraints(maxVel,maxAcc),
                startPoint, endPoint
        );
    }

    public static PathPlannerTrajectory getPathFromPose(Pose2d currPose, Pose2d goalPose, double maxVel, double maxAcc) {
        return getPathFromPose(currPose, 0, goalPose, maxVel, maxAcc);
    }

    /** gives desired heading in radians pointing from A to B */
    public static double calcHeading(Translation2d translationA, Translation2d translationB) {
        //Move the ending about the initial
        Translation2d aboutInitialTranslation = translationB.minus(translationA);
        //This is to find the reference angle and the heading in radians
        return Math.atan2(aboutInitialTranslation.getY(), aboutInitialTranslation.getX());
    }

    public static class PathWithName {
        public final String name;
        public final PathPlannerTrajectory path;

        public PathWithName(String name, PathPlannerTrajectory path) {
            this.name = name;
            this.path = path;
        }
    }
}
