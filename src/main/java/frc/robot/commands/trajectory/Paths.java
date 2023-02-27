package frc.robot.commands.trajectory;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DrivetrainBase;

import java.util.List;

import static com.pathplanner.lib.PathPlanner.*;

public final class Paths {
    public static final PathPlannerTrajectory straightPath = loadPath("Straight Path", 1, 0.5);
    public static final PathPlannerTrajectory straight180Path = loadPath("180 while forward", 1, 0.5);
    public static final PathPlannerTrajectory tPath = loadPath("TPath", 3, 1);
    public static final PathPlannerTrajectory circularPath = loadPath("Circular", 1, 0.5);
    public static PathPlannerTrajectory Auto1 = loadPath("Auto1", 1, 0.5);
    public static final PathPlannerTrajectory testPath = loadPath("Test", 1, 0.5);
    public static final PathPlannerTrajectory doubleConeChargingStationPath = loadPath("Double Cone Charging Station", 1, 0.5);
    public static final PathPlannerTrajectory diagonalPath = getPathToPose(
            new Pose2d(2.5, 1.5, Rotation2d.fromDegrees(-90)),
            2, 3);

    public static final PathPlannerTrajectory fivePath = PathPlanner.loadPath("5", 2, 3);
    public static final PathPlannerTrajectory fourPath = PathPlanner.loadPath("4", 2, 3);
    public static final PathPlannerTrajectory twoPath = PathPlanner.loadPath("2", 2, 3);

    static final PathPlannerTrajectory fiveToFourTransition =
            getPathToPose(
                new Pose2d(
                        fivePath.getEndState().poseMeters.getTranslation(),
                        fivePath.getEndState().holonomicRotation),
                fourPath.getInitialHolonomicPose(),
                0.7, 0.5
            );

    static final PathPlannerTrajectory fourToTwoTransition =
            getPathToPose(
                new Pose2d(
                        fourPath.getEndState().poseMeters.getTranslation(),
                        fourPath.getEndState().holonomicRotation),
                twoPath.getInitialHolonomicPose(),
                0.7, 0.5
            );

    static final PathPlannerTrajectory twoToTwoTransition =
            getPathToPose(
                new Pose2d(
                        twoPath.getEndState().poseMeters.getTranslation(),
                        twoPath.getEndState().holonomicRotation),
                twoPath.getInitialHolonomicPose(),
                0.7, 0.5
            );

    public static final List<PathWithName> listOfPaths = List.of(
            new PathWithName("Straight", straightPath),
            new PathWithName("Straight 180", straight180Path),
            new PathWithName("T", tPath),
            new PathWithName("Circular", circularPath),
            new PathWithName("Test", testPath),
            new PathWithName("Diagonal", diagonalPath),
            new PathWithName("Double Cone Charging Station", doubleConeChargingStationPath));

    public static PathPlannerTrajectory getPathToPose(Pose2d startPose, Pose2d endPose, double maxVel, double maxAcc) {
        return generatePath(
                    new PathConstraints(maxVel, maxAcc),
                    new PathPoint(startPose.getTranslation(), new Rotation2d(), startPose.getRotation()),
                    new PathPoint(endPose.getTranslation(), new Rotation2d(), endPose.getRotation()));
    }

    public static PathPlannerTrajectory getPathToPose(Pose2d endPose, double maxVel, double maxAcc) {
        return getPathToPose(new Pose2d(), endPose, maxVel, maxAcc);
    }

    /** end pose should be given relative to robot function will transform pose */
    public static PathPlannerTrajectory getPathRelativeToCurrentPose(Pose2d endPoseFromRobotPose, double maxVel, double maxAcc) {
        Pose2d currentPose = RobotState.getInstance().getCurrentPose();
        //transform is just the current pose - nothing
        var transform = currentPose.minus(new Pose2d());
        return getPathToPose(
                currentPose,
                endPoseFromRobotPose.transformBy(transform),
                maxVel, maxAcc
        );
    }

    public static PathPlannerTrajectory getPathAbsoluteToCurrentPose(Pose2d endFieldPose, double maxVel, double maxAcc) {
        Pose2d currentPose = RobotState.getInstance().getCurrentPose();
        return getPathToPose(currentPose, endFieldPose, maxVel, maxAcc);
    }

    public static SequentialCommandGroup getTeamNumberPathCommand(DrivetrainBase drivetrain) {
        return new SequentialCommandGroup(
                new PrintCommand("Starting team number command at position: " + RobotState.getInstance().getCurrentPose()),
                new FollowPathCommand(fivePath, drivetrain),
                new FollowPathCommand(fiveToFourTransition, drivetrain),
                new FollowPathCommand(fourPath, drivetrain),
                new FollowPathCommand(fourToTwoTransition, drivetrain),
                new FollowPathCommand(twoPath, drivetrain),
                new FollowPathCommand(twoToTwoTransition, drivetrain),
                new FollowPathCommand(twoPath, drivetrain)
        );
    }

    public static class PathWithName {
        public final String name;
        public final PathPlannerTrajectory path;
        public final Pose2d startPose;

        public PathWithName(String name, PathPlannerTrajectory path) {
            this.name = name;
            this.path = path;
            this.startPose = path.getInitialHolonomicPose();
        }
    }
}
