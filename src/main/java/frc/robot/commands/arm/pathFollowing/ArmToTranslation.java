package frc.robot.commands.arm.pathFollowing;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.arm.Arm;

import java.util.function.Supplier;

import static frc.robot.commands.drive.pathFollowing.Paths.calcHeading;

public class ArmToTranslation extends ArmPathFollowingCommand {

    private final Arm arm;
    private final Supplier<Translation2d> goalSupplier;
    private final double maxVel, maxAcc;

    public ArmToTranslation(Arm arm, Supplier<Translation2d> goalSupplier, double maxVel, double maxAcc) {
        super(arm);
        this.arm = arm;
        this.goalSupplier = goalSupplier;
        this.maxVel = maxVel;
        this.maxAcc = maxAcc;
    }

    public ArmToTranslation(Arm arm, Translation2d goalTranslation, double maxVel, double maxAcc) {
        this (arm, () -> goalTranslation, maxVel, maxAcc);
    }

    @Override
    public void initialize() {
        Translation2d current = arm.getGripperPose().getTranslation();
        Translation2d goal = goalSupplier.get();
        // if we start lower than the goal then we should do a different movement
        final boolean isLower = current.getY() <= goal.getY();
        Translation2d intermediate = isLower?
                new Translation2d(current.getX(), goal.getY()) :
                new Translation2d(goal.getX(), current.getY());

        // control radius of arc using divisor
        final double startControlLength = current.getDistance(intermediate) * 1.2;
        Rotation2d startHeading = new Rotation2d(calcHeading(current, intermediate));
        PathPoint start = new PathPoint(current, startHeading, new Rotation2d())
                .withNextControlLength(startControlLength + 0.01);

        final double endControlLength = goal.getDistance(intermediate) * 1.2;
        PathPoint end = new PathPoint(goal, new Rotation2d(calcHeading(intermediate, goal)), new Rotation2d())
                .withPrevControlLength(endControlLength + 0.01);

        var path = PathPlanner.generatePath(
                new PathConstraints(maxVel, maxAcc),
                start, end);
        addPath(path);
        super.initialize();
    }
}
