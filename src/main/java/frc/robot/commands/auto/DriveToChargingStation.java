package frc.robot.commands.auto;

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
    private static final double DISTANCE_TO_ENGAGE_STATION = Units.inchesToMeters(10);
    private static final double MARGIN = Units.inchesToMeters(30.0);

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

    private static Pose2d[] getNextPoses(Pose2d currentPose, RectangleRegion chargingStation) {
        double halfRobotWidth = (Constants.ROBOT_WIDTH + Constants.BUMPER_THICKNESS) / 2.0 + MARGIN;
        double halfRobotLength = (Constants.ROBOT_LENGTH + Constants.BUMPER_THICKNESS) / 2.0 + MARGIN
                + Constants.TURN_RADIUS;

        double minY = chargingStation.minY + halfRobotLength;
        double maxY = chargingStation.maxY - halfRobotWidth;
        double alignedY = MathUtil.clamp(currentPose.getY(), minY, maxY);

        boolean isRightOfStation = currentPose.getX() >= chargingStation.getCenter().getX();
        double alignedX = isRightOfStation?
                chargingStation.maxX + halfRobotLength : chargingStation.minX - halfRobotLength;

        Rotation2d rotation = isRightOfStation? new Rotation2d(Math.PI) : new Rotation2d();

        ArrayList<Pose2d> poses = new ArrayList<>();
        if (currentPose.getX() <= chargingStation.maxX && currentPose.getX() >= chargingStation.minX) {
            poses.add(new Pose2d(alignedX, currentPose.getY(), currentPose.getRotation()));
        }
        poses.add(new Pose2d(alignedX, alignedY, rotation));
        double engagedX = (isRightOfStation? -1.0 : 1.0) * DISTANCE_TO_ENGAGE_STATION + alignedX;
        poses.add(new Pose2d(engagedX, alignedY, rotation));

        Pose2d[] poseArray = new Pose2d[poses.size()];
        return poses.toArray(poseArray);
    }
}
