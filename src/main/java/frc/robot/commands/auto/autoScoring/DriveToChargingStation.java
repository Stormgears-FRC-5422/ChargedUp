package frc.robot.commands.auto.autoScoring;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotState;
import frc.robot.commands.drive.pathFollowing.PathFollowingCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.DrivetrainBase;

import java.util.ArrayList;
import java.util.List;

import static frc.robot.commands.drive.pathFollowing.Paths.calcHeading;

import static frc.robot.constants.FieldConstants.Regions.*;

public class DriveToChargingStation extends PathFollowingCommand {

    RectangleRegion chargingStation = FieldConstants.getChargingStation();
    private static final double DISTANCE_TO_ENGAGE_STATION = Units.inchesToMeters(25.0);
    private static final double MARGIN = Units.inchesToMeters(3.0);

    public DriveToChargingStation(DrivetrainBase drivetrain) {
        super(drivetrain);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = RobotState.getInstance().getCurrentPose();
        var points = getNextPoses(currentPose, chargingStation);
        var path = PathPlanner.generatePath(
                new PathConstraints(1.5, 1), points);
        addPath(path);
        super.initialize();
    }

    private static List<PathPoint> getNextPoses(Pose2d currentPose, RectangleRegion station) {
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
        Rotation2d alignedRotation;
        if (currentDegrees < 0)
            alignedRotation = (currentDegrees < -90.0)? new Rotation2d(Math.PI) : new Rotation2d();
        else
            alignedRotation = (currentDegrees > 90.0)? new Rotation2d(Math.PI) : new Rotation2d();

        // give an array of path points
        ArrayList<PathPoint> points = new ArrayList<>();

        PathPoint initial = null;
        boolean isRightOfStation = currentPose.getX() >= station.getCenter().getX();
        double alignedX = isRightOfStation?
                station.maxX + ALIGN_MARGIN : station.minX - ALIGN_MARGIN;
        Rotation2d initialHeading = new Rotation2d(isRightOfStation? 0.0 : Math.PI);
        if (currentPose.getX() <= station.maxX && currentPose.getX() <= station.minX) {
            initial = new PathPoint(
                    new Translation2d(alignedX, currentPose.getY()),
                    initialHeading,
                    currentPose.getRotation()
            );
        }

        Rotation2d alignedHeading = new Rotation2d(isRightOfStation? Math.PI : 0.0);
        Translation2d alignedTranslation = new Translation2d(alignedX, alignedY);
        PathPoint aligned = new PathPoint(
                alignedTranslation,
                alignedHeading,
                alignedRotation
        ).withPrevControlLength(0.1);

        double engagedX = (isRightOfStation? -1.0 : 1.0) * DISTANCE_TO_ENGAGE_STATION + alignedX;
        PathPoint engaged = new PathPoint(
                new Translation2d(engagedX, alignedY),
                alignedHeading,
                alignedRotation
        );

        boolean useInitial = initial != null;
        PathPoint current = new PathPoint(
                currentPose.getTranslation(),
                useInitial?
                        initialHeading :
                        new Rotation2d(calcHeading(currentPose.getTranslation(), alignedTranslation)),
                currentPose.getRotation()
        );

        return useInitial?
                List.of(current, initial, aligned, engaged) :
                List.of(current, aligned, engaged);
    }
}