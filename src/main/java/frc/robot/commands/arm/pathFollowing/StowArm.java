package frc.robot.commands.arm.pathFollowing;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.Constants;
import frc.robot.subsystems.arm.Arm;

public class StowArm extends ArmPathFollowingCommand {

    private final Arm arm;

    public StowArm(Arm arm) {
        super(arm);
        this.arm = arm;
    }

    @Override
    public void initialize() {
        Translation2d current = arm.getGripperPose().getTranslation();

        PathPoint startPoint = new PathPoint(
                current,
                new Rotation2d(Math.PI),
                new Rotation2d()
        ).withNextControlLength(0.5);

        PathPoint endPoint = new PathPoint(
                Constants.ArmConstants.stowPosition,
                new Rotation2d(-Math.PI),
                new Rotation2d()
        ).withPrevControlLength(0.5);

        var path = PathPlanner.generatePath(
                new PathConstraints(3, 3),
                startPoint, endPoint
        );

        addPath(path);
        super.initialize();
    }
}
