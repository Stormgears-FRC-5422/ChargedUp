package frc.robot.commands.autoScoring;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotState;
import frc.robot.commands.drive.pathFollowing.PathFollowingCommand;
import frc.robot.commands.drive.pathFollowing.Paths;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.DrivetrainBase;

import java.util.ArrayList;

import static frc.robot.constants.FieldConstants.Regions.*;

public class DriveToChargingStation extends PathFollowingCommand {

    RectangleRegion chargingStation = getCurrentChargingStation();
    private static final double DISTANCE_TO_ENGAGE_STATION = Units.inchesToMeters(25.0);
    private static final double MARGIN = Units.inchesToMeters(3.0);

    public DriveToChargingStation(DrivetrainBase drivetrain) {
        super(drivetrain);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = RobotState.getInstance().getCurrentPose();
        var poses = getNextPoses(currentPose, chargingStation);
        var path = Paths.getPathFromHolonomicPoses(2.0, 1.5, poses);
        withPath(path);
        super.initialize();
    }

    private static Pose2d[] getNextPoses(Pose2d currentPose, RectangleRegion station) {
        double halfRobotWidth = (Constants.ROBOT_WIDTH + Constants.BUMPER_THICKNESS) / 2.0 + MARGIN;
        double ALIGN_MARGIN = Constants.TURN_RADIUS + MARGIN;

        // get the y coordinate of our robot our if it is not aligned with chargin station
        // get the minimum or maximum field y we have to be at
        double minY = station.minY + halfRobotWidth;
        double maxY = station.maxY - halfRobotWidth;
        double alignedY = MathUtil.clamp(currentPose.getY(), minY, maxY);

        // either get onto the charging station forwards or backwards
        // depending on which is closer
        double currentDegrees = currentPose.getRotation().getDegrees();
        Rotation2d rotation;
        if (currentDegrees < 0)
            rotation = (currentDegrees < -90.0)? new Rotation2d(Math.PI) : new Rotation2d();
        else
            rotation = (currentDegrees > 90.0)? new Rotation2d(Math.PI) : new Rotation2d();

        // give an array of poses
        ArrayList<Pose2d> poses = new ArrayList<>();
        // find which sid eof station we are on
        boolean isRightOfStation = currentPose.getX() >= station.getCenter().getX();
        double alignedX = isRightOfStation?
                station.maxX + ALIGN_MARGIN : station.minX - ALIGN_MARGIN;
        // if we are somewhere in the middle the first step should be to go to one of the sides
        if (currentPose.getX() <= station.maxX && currentPose.getX() >= station.minX) {
            poses.add(new Pose2d(alignedX, currentPose.getY(), currentPose.getRotation()));
        }
        // aligned position
        poses.add(new Pose2d(alignedX, alignedY, rotation));
        // determine which way to travel for engagement with station
        double engagedX = (isRightOfStation? -1.0 : 1.0) * DISTANCE_TO_ENGAGE_STATION + alignedX;
        poses.add(new Pose2d(engagedX, alignedY, rotation));

        Pose2d[] poseArray = new Pose2d[poses.size()];
        return poses.toArray(poseArray);
    }
}
