package frc.robot.commands.auto.autoManeuvers;

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
import frc.robot.subsystems.drive.DrivetrainBase;

import static frc.robot.commands.auto.autoManeuvers.DriveToDoubleSubstation.Position.LEFT;
import static frc.robot.constants.FieldConstants.Substations.*;
import static frc.robot.commands.drive.pathFollowing.Paths.calcHeading;

public class DriveToDoubleSubstation extends PathFollowingCommand {
    private final Position side;

    public DriveToDoubleSubstation(DrivetrainBase drivetrain, Position side) {
        super(drivetrain);

        this.side = side;
    }

    @Override
    public void initialize() {
        Pose2d currentPose = RobotState.getInstance().getCurrentPose();
        double currentVel = RobotState.getInstance().getCurrentLinearVel();
        DoubleSubstation currentSubstation = getDoubleSubstation();
        boolean isRed = DriverStation.getAlliance() == DriverStation.Alliance.Red;

        Translation2d goalTranslation = (side == LEFT)?
                currentSubstation.getLeftTranslation(currentPose.getTranslation()) :
                currentSubstation.getRightTranslation(currentPose.getTranslation());

        Rotation2d goalRotation = isRed?
                new Rotation2d(Math.PI) : new Rotation2d();

        final double alignMeters = Units.inchesToMeters(10.0);

        Translation2d intermediate = isRed?
                new Translation2d(goalTranslation.getX(), goalTranslation.getY() + alignMeters) :
                new Translation2d(goalTranslation.getX(), goalTranslation.getY() - alignMeters);

        Rotation2d firstHeading = new Rotation2d(calcHeading(currentPose.getTranslation(), intermediate));
        PathPoint start = new PathPoint(currentPose.getTranslation(), firstHeading, currentPose.getRotation(), currentVel);
        PathPoint end = new PathPoint(goalTranslation, goalRotation, goalRotation);

        PathPlannerTrajectory path = PathPlanner.generatePath(
                new PathConstraints(1.0, 1.0), start, end);
        setPath(path);
        super.initialize();
    }

    public enum Position {
        LEFT, RIGHT, FLOOR, NONE
    }
}
