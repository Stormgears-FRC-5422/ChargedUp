package frc.robot.commands.trajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import frc.robot.RobotState;

import javax.xml.crypto.dsig.Transform;
import java.io.IOException;
import java.nio.file.Path;
import java.util.*;

public final class Trajectories {

    public static Trajectory straightLineNoTurn(double maxVelocity, double maxAcceleration, SwerveDriveKinematics kinematics) {
        //Config for trajectory
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(maxVelocity, maxAcceleration);
        trajectoryConfig.setStartVelocity(RobotState.getInstance().getCurrentLinearVel());
        trajectoryConfig.setKinematics(kinematics);
        //return actual trajectory very crude
        Pose2d currentPose = RobotState.getInstance().getCurrentPose();
        //get points
        Pose2d startPose = new Pose2d(0, 0, new Rotation2d(0));
        var interiorWaypoints = List.of(new Translation2d(1.5, -1));
        Pose2d endPose = new Pose2d(3, 0, new Rotation2d(0));
        //generate trajectory
        var outputTrajectory = TrajectoryGenerator.generateTrajectory(
                startPose,
                interiorWaypoints,
                endPose,
                trajectoryConfig
        );
        //translation is current pose (translation and rotation)
        Transform2d transformByCurrent = currentPose.minus(new Pose2d());
        return outputTrajectory.transformBy(transformByCurrent);
    }

    public static Trajectory straightLineWhileTurn(double goalAngle, double maxVelocity, double maxAcceleration, SwerveDriveKinematics kinematics) {
        //Config for trajectory
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(maxVelocity, maxAcceleration);
        trajectoryConfig.setStartVelocity(RobotState.getInstance().getCurrentLinearVel());
        trajectoryConfig.setKinematics(kinematics);
        //return actual trajectory very crude
        Pose2d currentPose = RobotState.getInstance().getCurrentPose();
        //get points
        Pose2d startPose = new Pose2d(0, 0, new Rotation2d(0));
        var interiorWaypoints = List.of(new Translation2d(1.5, 0));
        Pose2d endPose = new Pose2d(3, 0, new Rotation2d((goalAngle/360.) * 2 * Math.PI));
        //generate trajectory
        var outputTrajectory = TrajectoryGenerator.generateTrajectory(
                startPose,
                interiorWaypoints,
                endPose,
                trajectoryConfig
        );
        Transform2d transformByCurrent = currentPose.minus(new Pose2d());
        return outputTrajectory.transformBy(transformByCurrent);
    }

    public static Trajectory fromJSON(Path JSONPath) throws IOException {
        //generate trajectory
        var outputTrajectory = TrajectoryUtil.fromPathweaverJson(JSONPath);
        Pose2d currentPose = RobotState.getInstance().getCurrentPose();
        Transform2d transformation = currentPose.minus(outputTrajectory.getInitialPose());
        return outputTrajectory.transformBy(transformation);
    }
}
