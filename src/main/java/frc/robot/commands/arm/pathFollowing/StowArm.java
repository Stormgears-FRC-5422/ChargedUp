package frc.robot.commands.arm.pathFollowing;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.Constants;
import frc.robot.subsystems.arm.Arm;

import static frc.robot.constants.Constants.ArmConstants.stowPosition;
import static frc.robot.commands.drive.pathFollowing.Paths.calcHeading;

public class StowArm extends ArmPathFollowingCommand {

    private final Arm arm;

    public StowArm(Arm arm) {
        super(arm);
        this.arm = arm;
    }

    @Override
    public void initialize() {
        Translation2d current = arm.getGripperPose().getTranslation();
        boolean useQuickPath = stowPosition.getDistance(current) <= 0.20 &&
                                current.getY() >= stowPosition.getY();

        Rotation2d startHeading = (useQuickPath) ?
            new Rotation2d(calcHeading(current, stowPosition)) : new Rotation2d(Math.PI / 2.0);
        PathPoint startPoint = new PathPoint(
                current,
                startHeading,
                new Rotation2d()
                ).withNextControlLength((useQuickPath)? current.getDistance(stowPosition) : 1.0);

        Rotation2d endHeading = (useQuickPath) ?
                startHeading : new Rotation2d(-Math.PI / 2.0);
        PathPoint endPoint = new PathPoint(
                stowPosition,
                endHeading,
                new Rotation2d()
        ).withPrevControlLength((useQuickPath)? current.getDistance(stowPosition) : 0.5);

        var path = PathPlanner.generatePath(
                new PathConstraints(4, 4),
                startPoint, endPoint
        );

        addPath(path);
        super.initialize();
    }
}
