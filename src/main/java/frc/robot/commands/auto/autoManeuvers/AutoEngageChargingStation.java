package frc.robot.commands.auto.autoManeuvers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotState;
import frc.robot.commands.drive.pathFollowing.PathFollowingCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.DrivetrainBase;

import static frc.robot.constants.FieldConstants.Regions.RectangleRegion;

public class AutoEngageChargingStation extends PathFollowingCommand {

    final double yMargin = Constants.ROBOT_WIDTH / 2.0 + Units.inchesToMeters(10.0);
    final double xAlignMargin = Constants.ROBOT_LENGTH / 2.0 + Units.inchesToMeters(5.0);

    public AutoEngageChargingStation(DrivetrainBase drivetrain) {
        super(drivetrain);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = RobotState.getInstance().getCurrentPose();
        RectangleRegion chargingStation = FieldConstants.getChargingStation();

        // calculate the end y position which is somewhat away from the edges
        double goalY = calculateGoalY(currentPose.getY(), chargingStation.minY, chargingStation.maxY);

        final boolean isRightOfChargingStation = currentPose.getX() >= chargingStation.getCenter().getX();

        super.initialize();
    }

    private double calculateGoalY(double current, double min, double max) {
        double safeMin = min + yMargin;
        double safeMax = max - yMargin;

        return MathUtil.clamp(current, safeMin, safeMax);
    }
}
