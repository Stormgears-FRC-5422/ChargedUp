package frc.robot.commands.auto.autoManeuvers;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.arm.pathFollowing.ArmPathFollowingCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.arm.Arm;

import java.util.function.Supplier;

import static frc.robot.constants.FieldConstants.Grids.ScoringNode;
import static frc.robot.commands.drive.pathFollowing.Paths.calcHeading;

public class ArmToNode extends ArmPathFollowingCommand {

    // TODO: find x and y positions for each node {high, mid, hybrid}
    private static final double[] goalXPositions = {1.15, 0.984, 0.5};


    private static final double CONE_OFFSET = Units.inchesToMeters(10.0);
    private static final double CUBE_OFFSET = Units.inchesToMeters(8.0);
    private static final double CONE_HIGH_OFFSET = 0.10189;
    private static final double CONE_MID_OFFSET = 0.14655;

    private final Arm arm;
    private final Supplier<ScoringNode> nodeSupplier;
    private Translation2d goal;
    private Timer timer = new Timer();

    public ArmToNode(Arm arm, Supplier<ScoringNode> nodeSupplier) {
        super(arm);
        this.arm = arm;
        this.nodeSupplier = nodeSupplier;
    }

    @Override
    public void initialize() {
        ScoringNode node = nodeSupplier.get();
        Translation3d nodeTranslation = node.translation;
        System.out.println("Moving arm to score on " + node.height + " of type " + node.type);
        final boolean isCube = node.type == ScoringNode.NodeType.CUBE;
        final boolean isCone = node.type == ScoringNode.NodeType.CONE;

        // added 4.0 inches due to testing
        double goalX = Math.abs(node.scoringPosition.getX() - nodeTranslation.getX());
        boolean high = node.height == ScoringNode.NodeHeight.HIGH;
        boolean mid = node.height == ScoringNode.NodeHeight.MIDDLE;
        boolean hybrid = node.height == ScoringNode.NodeHeight.HYBRID;


        double goalZ = nodeTranslation.getZ();

        if (high) {
            goalZ += ((isCone) ? CONE_HIGH_OFFSET : 0);
            goalX -= ((isCube) ? 0 : 0.08451);
        } else if (mid) {
            goalZ += ((isCone) ? CONE_MID_OFFSET : 0);
            goalX -= ((isCube) ? 0.04888 : 0.50777);
        } else if (hybrid) {
            goalX -= 0.25598;        }

//        if (mid && node.type == ScoringNode.NodeType.CONE)
//            goalZ -= Units.inchesToMeters(3.5);

        Translation3d globalCoordinate = new Translation3d(
                goalX, 0, goalZ
        );
        goal = Arm.fromGlobalTranslation(globalCoordinate);
        double goalY = goal.getY();

        Translation2d current = arm.getGripperPose().getTranslation();
        double deltaY = MathUtil.clamp(goalY - current.getY(), 0.01, 2.0);

        // If it is a hybrid then decrease the arc from straight up to Math.PI / n
        Rotation2d startHeading = hybrid?
                new Rotation2d(Math.PI / 3) : new Rotation2d(Math.PI / 2);
        PathPoint start = new PathPoint(current, startHeading, new Rotation2d())
                .withNextControlLength(hybrid? Units.inchesToMeters(14.5) : deltaY * 1.2);

        PathPoint end = new PathPoint(goal, new Rotation2d(-Math.PI / 2.0), new Rotation2d())
                .withPrevControlLength(hybrid? 0.3 : 1.5);

        // not changing speeds for cube or cone put condition back in if placing cones
        // we are only placing cubes
        final var constraints = new PathConstraints(8.0, 10.0);
        var path = PathPlanner.generatePath(
                constraints,
                start, end);
        addPath(path);
        super.initialize();
        timer.reset();
        timer.start();
    }

    public Translation2d getGoalTarget() {
        if (goal == null)
            return new Translation2d();
        return goal;
    }

    @Override
    public boolean isFinished() {
        if (nodeSupplier.get().height != ScoringNode.NodeHeight.HYBRID)
            return super.isFinished();
        return timer.hasElapsed(1.2);
    }
}
