package frc.robot.commands.drive.pathFollowing;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotState;

import java.util.ArrayList;
import java.util.List;

import static frc.robot.constants.FieldConstants.*;
import static com.pathplanner.lib.PathPlanner.*;
import static com.pathplanner.lib.PathPlannerTrajectory.*;

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

    public static PathPlannerTrajectory getPathFromHolonomicPoses(double maxVel, double maxAcc, Pose2d... poses) {
        ArrayList<PathPoint> points = new ArrayList<>();
        for (int i = 0; i < poses.length; i++) {
            double heading;
            if (i == poses.length - 1)
                heading = calcHeading(poses[i-1].getTranslation(), poses[i].getTranslation());
            else
                heading = calcHeading(poses[i].getTranslation(), poses[i+1].getTranslation());
            points.add(new PathPoint(
                    poses[i].getTranslation(),
                    new Rotation2d(heading),
                    poses[i].getRotation()
            ).withControlLengths(0.01, 0.01));
        }
        return PathPlanner.generatePath(
                new PathConstraints(maxVel, maxAcc), points);
    }

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

    public static PathPlannerState mirrorState(
            PathPlannerState state,
            DriverStation.Alliance alliance) {
        if (alliance == DriverStation.Alliance.Blue) {
            System.out.println("Called flipState() on the blue allliance");
            return state;
        }

        Translation2d mirroredTranslation = mirrorTranslation(state.poseMeters.getTranslation());
        Rotation2d mirroredHeading = mirrorRotation(state.poseMeters.getRotation());
        Rotation2d mirroredHolonomicRotation = mirrorRotation(state.holonomicRotation);

        var mirroredState = transformStateForAlliance(state, alliance);
        mirroredState.poseMeters = new Pose2d(mirroredTranslation, mirroredHeading);
        mirroredState.holonomicRotation = mirroredHolonomicRotation;

        return mirroredState;
    }

    public static PathPlannerTrajectory mirrorPath(
            PathPlannerTrajectory path,
            DriverStation.Alliance alliance
    ) {
        if (alliance == DriverStation.Alliance.Blue) {
            System.out.println("Called mirrorPath() on blue alliance");
            return path;
        }

        List<State> mirroredStates = new ArrayList<>();
        for (State s : path.getStates()) {
            PathPlannerState state = (PathPlannerState) s;
            mirroredStates.add(mirrorState(state, alliance));
        }

        return new PathPlannerTrajectory(
                mirroredStates,
                path.getMarkers(),
                path.getStartStopEvent(),
                path.getEndStopEvent(),
                path.fromGUI
        );
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
